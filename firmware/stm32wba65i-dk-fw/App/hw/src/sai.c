#include "sai.h"



#ifdef _USE_HW_SAI
#include "cli.h"
#include "gpio.h"
#include "qbuffer.h"
#include "buzzer.h"
#include "mixer.h"



#define SAI_SAMPLERATE_HZ       16000
#define SAI_BUF_MS              (4)
#define SAI_BUF_LEN             (8*1024)
#define SAI_BUF_FRAME_LEN       ((48000 * 2 * SAI_BUF_MS) / 1000)  // 48Khz, Stereo, 4ms


typedef struct
{
  int16_t volume;
} sai_cfg_t;


#ifdef _USE_HW_CLI
static void cliSai(cli_args_t *args);
#endif

static bool is_init = false;
static bool is_started = false;
static bool is_busy = false;

static uint32_t sai_sample_rate = SAI_SAMPLERATE_HZ;

static int16_t  sai_frame_buf[SAI_BUF_FRAME_LEN * 2];
static uint32_t sai_frame_len = 0;

static mixer_t   mixer;
static int16_t   sai_volume = 100;
static sai_cfg_t sai_cfg;

extern SAI_HandleTypeDef hsai_BlockA1;







bool saiInit(void)
{
  bool ret = true;


  hsai_BlockA1.Instance             = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode       = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro         = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive     = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider       = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold   = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency  = SAI_AUDIO_FREQUENCY_16K;
  hsai_BlockA1.Init.SynchroExt      = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MckOutput       = SAI_MCK_OUTPUT_ENABLE;
  hsai_BlockA1.Init.MonoStereoMode  = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode  = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState        = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    ret = false;
  }

  sai_frame_len = (sai_sample_rate * 2 * SAI_BUF_MS) / 1000;
  
  mixerInit(&mixer);
  saiCfgLoad();

  saiStart();

  delay(50);

  is_init = ret;

  logPrintf("[%s] saiInit()\n", ret ? "OK" : "NG");


  #ifdef _USE_HW_CLI
  cliAdd("sai", cliSai);
#endif

  return ret;
}

bool saiCfgLoad(void)
{
  bool ret = true;

  #ifdef _USE_HW_NVS
  if (nvsGet(SAI_CFG_NAME, &sai_cfg, sizeof(sai_cfg)) == true)
  {
    sai_volume = sai_cfg.volume;
  }
  else
  {
    sai_cfg.volume = sai_volume;
    ret = nvsSet(SAI_CFG_NAME, &sai_cfg, sizeof(sai_cfg));
    logPrintf("[NG] saiCfgLoad()\n");
  }
  #else
  sai_cfg.volume = sai_volume;
  #endif

  saiSetVolume(sai_cfg.volume);
  return ret;
}

bool saiCfgSave(void)
{
  bool ret = true;

  sai_cfg.volume = sai_volume;

  #ifdef _USE_HW_NVS
  ret = nvsSet(SAI_CFG_NAME, &sai_cfg, sizeof(sai_cfg));
  #endif
  
  return ret;
}

bool saiIsBusy(void)
{
  return is_busy;
}

bool saiSetSampleRate(uint32_t freq)
{
  bool ret = true;
  uint32_t frame_len;
  const uint32_t freq_tbl[7] = 
  {
    SAI_AUDIO_FREQUENCY_48K,  
    SAI_AUDIO_FREQUENCY_44K,
    SAI_AUDIO_FREQUENCY_32K,
    SAI_AUDIO_FREQUENCY_22K,
    SAI_AUDIO_FREQUENCY_16K,
    SAI_AUDIO_FREQUENCY_11K,
    SAI_AUDIO_FREQUENCY_8K,
  };

  ret = false;
  for (int i=0; i<7; i++)
  {
    if (freq_tbl[i] == freq)
    {
      ret = true;
      break;
    }
  }
  if (ret != true)
  {
    return false;
  }

  saiStop();
  delay(10);

  sai_sample_rate = freq;
  frame_len = (sai_sample_rate * 2 * SAI_BUF_MS) / 1000;
  sai_frame_len = frame_len;


  hsai_BlockA1.Init.AudioFrequency  = freq;
  HAL_SAI_MspInit(&hsai_BlockA1);
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    ret = false;
  }
  HAL_SAI_MspInit(&hsai_BlockA1);

  saiStart();

  return ret;
}

uint32_t saiGetSampleRate(void)
{
  return sai_sample_rate;
}

bool saiStart(void)
{
  bool ret = false;
  HAL_StatusTypeDef status;

  memset(sai_frame_buf, 0, sizeof(sai_frame_buf));
  status = HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)sai_frame_buf, sai_frame_len * 2);
  if (status == HAL_OK)
  {
    is_started = true;
  }
  else
  {
    is_started = false;
  }

  return ret;
}

bool saiStop(void)
{
  is_started = false;
  return true;
}

int8_t saiGetEmptyChannel(void)
{
  return mixerGetEmptyChannel(&mixer);
}

uint32_t saiGetFrameSize(void)
{
  return sai_frame_len;
}

uint32_t saiAvailableForWrite(uint8_t ch)
{
  return mixerAvailableForWrite(&mixer, ch);
}

bool saiWrite(uint8_t ch, int16_t *p_data, uint32_t length)
{
  return mixerWrite(&mixer, ch, p_data, length);
}

// https://m.blog.naver.com/PostView.nhn?blogId=hojoon108&logNo=80145019745&proxyReferer=https:%2F%2Fwww.google.com%2F
//
float saiGetNoteHz(int8_t octave, int8_t note)
{
  float hz;
  float f_note;

  if (octave < 1) octave = 1;
  if (octave > 8) octave = 8;

  if (note <  1) note = 1;
  if (note > 12) note = 12;

  f_note = (float)(note-10)/12.0f;

  hz = pow(2, (octave-1)) * 55 * pow(2, f_note);

  return hz;
}

// https://gamedev.stackexchange.com/questions/4779/is-there-a-faster-sine-function
//
float saiSin(float x)
{
  const float B = 4 / M_PI;
  const float C = -4 / (M_PI * M_PI);

  return -(B * x + C * x * ((x < 0) ? -x : x));
}

bool saiPlayBeep(uint32_t freq_hz, uint16_t volume, uint32_t time_ms)
{
  uint32_t pre_time;
  int32_t sample_rate = sai_sample_rate;
  int32_t num_samples = sai_frame_len;
  float sample_point;
  int16_t sample[num_samples];
  int16_t sample_index = 0;
  float div_freq;
  int8_t mix_ch;
  int32_t volume_out;


  volume = constrain(volume, 0, 100);
  volume_out = (INT16_MAX/40) * volume / 100;

  mix_ch =  saiGetEmptyChannel();

  div_freq = (float)sample_rate/(float)freq_hz;

  pre_time = millis();
  while(millis()-pre_time <= time_ms)
  {
    if (saiAvailableForWrite(mix_ch) >= num_samples)
    {
      for (int i=0; i<num_samples; i+=2)
      {
        sample_point = saiSin(2.0f * M_PI * (float)(sample_index) / ((float)div_freq));
        sample[i + 0] = (int16_t)(sample_point * volume_out);
        sample[i + 1] = (int16_t)(sample_point * volume_out);
        sample_index = (sample_index + 1) % (int)(div_freq);
      }
      saiWrite(mix_ch, (int16_t *)sample, num_samples);
    }
    delay(2);
  }

  return true;
}

int16_t saiGetVolume(void)
{
  return sai_volume;
}

bool saiSetVolume(int16_t volume)
{
  volume = constrain(volume, 0, 100);
  sai_volume = volume;
  mixerSetVolume(&mixer, sai_volume);
  return true;
}

void i2sUpdateBuffer(uint8_t index)
{
  if (mixerAvailable(&mixer) >= sai_frame_len)
  {    
    mixerRead(&mixer, &sai_frame_buf[index * sai_frame_len], sai_frame_len);
    is_busy = true;
  }
  else
  {
    memset(&sai_frame_buf[index * sai_frame_len], 0, sai_frame_len * 2);
    is_busy = false;
  }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hi2s)
{
  UNUSED(hi2s);
  i2sUpdateBuffer(0);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hi2s)
{
  UNUSED(hi2s);
  i2sUpdateBuffer(1);
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hi2s)
{
  UNUSED(hi2s);
}


#ifdef _USE_HW_CLI
typedef struct wavfile_header_s
{
  char    ChunkID[4];     /*  4   */
  int32_t ChunkSize;      /*  4   */
  char    Format[4];      /*  4   */

  char    Subchunk1ID[4]; /*  4   */
  int32_t Subchunk1Size;  /*  4   */
  int16_t AudioFormat;    /*  2   */
  int16_t NumChannels;    /*  2   */
  int32_t SampleRate;     /*  4   */
  int32_t ByteRate;       /*  4   */
  int16_t BlockAlign;     /*  2   */
  int16_t BitsPerSample;  /*  2   */

  char    Subchunk2ID[4];
  int32_t Subchunk2Size;
} wavfile_header_t;


void cliSai(cli_args_t *args)
{
  bool ret = false;



  if (args->argc == 1 && args->isStr(0, "info"))
  {

    cliPrintf("sai init      : %d\n", is_init);
    cliPrintf("sai rate      : %d Khz\n", sai_sample_rate/1000);
    cliPrintf("sai buf ms    : %d ms\n", SAI_BUF_MS);
    cliPrintf("sai frame len : %d \n", sai_frame_len);   
    cliPrintf("sai volume    : %d\n", saiGetVolume()); 
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "volume"))
  {
    cliPrintf("sai volume: %d\n", saiGetVolume()); 
    ret = true;
  }

  if (args->argc == 2 && args->isStr(0, "volume"))
  {
    int16_t volume;

    volume = args->getData(1);
    
    cliPrintf("cur volume: %d\n", saiGetVolume()); 

    saiSetVolume(volume);
    saiCfgSave();
    
    cliPrintf("set volume: %d\n", saiGetVolume()); 
    ret = true;
  }

  if (args->argc == 3 && args->isStr(0, "beep"))
  {
    uint32_t freq;
    uint32_t time_ms;

    freq = args->getData(1);
    time_ms = args->getData(2);
    
    saiPlayBeep(freq, 100, time_ms);

    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "melody"))
  {
    uint16_t melody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};
    int note_durations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };

    for (int i=0; i<8; i++) 
    {
      int note_duration = 1000 / note_durations[i];

      saiPlayBeep(melody[i], 100, note_duration);
      delay(note_duration * 0.3);    
    }
    ret = true;
  }

  if (args->argc == 2 && args->isStr(0, "play-wav") == true)
  {
    char *file_name;
    FILE *fp;
    wavfile_header_t header;
    uint32_t r_len;
    int32_t  volume = 100;
    int8_t ch;


    file_name = args->getStr(1);

    cliPrintf("FileName      : %s\n", file_name);


    fp = fopen(file_name, "r");
    if (fp == NULL)
    {
      cliPrintf("fopen fail : %s\n", file_name);
      return;
    }
    fread(&header, sizeof(wavfile_header_t), 1, fp);

    cliPrintf("ChunkSize     : %d\n", header.ChunkSize);
    cliPrintf("Format        : %c%c%c%c\n", header.Format[0], header.Format[1], header.Format[2], header.Format[3]);
    cliPrintf("Subchunk1Size : %d\n", header.Subchunk1Size);
    cliPrintf("AudioFormat   : %d\n", header.AudioFormat);
    cliPrintf("NumChannels   : %d\n", header.NumChannels);
    cliPrintf("SampleRate    : %d\n", header.SampleRate);
    cliPrintf("ByteRate      : %d\n", header.ByteRate);
    cliPrintf("BlockAlign    : %d\n", header.BlockAlign);
    cliPrintf("BitsPerSample : %d\n", header.BitsPerSample);
    cliPrintf("Subchunk2Size : %d\n", header.Subchunk2Size);


    saiSetSampleRate(header.SampleRate);

    r_len = sai_frame_len/2;

    int16_t buf_frame[sai_frame_len];

    fseek(fp, sizeof(wavfile_header_t) + 1024, SEEK_SET);

    ch = saiGetEmptyChannel();

    while(cliKeepLoop())
    {
      int len;


      if (saiAvailableForWrite(ch) >= sai_frame_len)
      {
        len = fread(buf_frame, r_len, 2*header.NumChannels, fp);

        if (len != (int)(r_len*2*header.NumChannels))
        {
          break;
        }

        int16_t buf_data[2];

        for (int i=0; i<(int)r_len; i++)
        {
          if (header.NumChannels == 2)
          {
            buf_data[0] = buf_frame[i*2 + 0] * volume / 100;;
            buf_data[1] = buf_frame[i*2 + 1] * volume / 100;;
          }
          else
          {
            buf_data[0] = buf_frame[i] * volume / 100;;
            buf_data[1] = buf_frame[i] * volume / 100;;
          }

          saiWrite(ch, (int16_t *)buf_data, 2);
        }
      }
    }

    fclose(fp);

    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("sai info\n");
    cliPrintf("sai volume [0~100]\n");
    cliPrintf("sai melody\n");
    cliPrintf("sai beep freq time_ms\n");
    cliPrintf("sai play-wav filename\n");
  }
}
#endif

#endif
