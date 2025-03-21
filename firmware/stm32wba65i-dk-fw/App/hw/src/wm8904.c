#include "wm8904.h"


#ifdef _USE_HW_WM8904
#include "cli.h"
#include "i2c.h"


#ifdef _USE_HW_CLI
static void cliCmd(cli_args_t *args);
#endif
static bool wm8904InitReg(void);
static bool wm8904ReadReg(uint8_t addr, uint16_t *p_data);
static bool wm8904WriteReg(uint8_t addr, uint16_t data);

static bool    is_init  = false;
static bool    is_found = false;
static uint8_t i2c_ch   = _DEF_I2C1;
static uint8_t i2c_addr = 0x1A;





bool wm8904Init(void)
{
  bool ret = true;


  if (!i2cIsBegin(i2c_ch))
  {
    ret = i2cBegin(i2c_ch, 400);    
  }

  if (ret)
  {
    if (i2cIsDeviceReady(i2c_ch, i2c_addr))
    {
      is_found = true;
      logPrintf("[OK] wm8904 Found\n");
      ret &= wm8904InitReg();
    }
    else
    {
      ret = false;
      logPrintf("[E_] wm8904 Not Found\n");
    }
  }

  is_init = ret;

  #ifdef _USE_HW_CLI
  cliAdd("wm8904", cliCmd);
  #endif  
  return ret;
}

bool wm8904InitReg(void)
{
  bool ret = true;
  uint16_t reg_data;
  uint32_t pre_time;


  // Reset
  //
  wm8904WriteReg(0x00, 0x0000); 
  delay(10);


  // Start-Up sequence
  //
  ret &= wm8904WriteReg(0x16, 0x0004); 

  ret &= wm8904WriteReg(0x6C, 0x0100); 
  ret &= wm8904WriteReg(0x6F, 0x0100); 
  ret &= wm8904WriteReg(0x21, 0x0000); 

  pre_time = millis();
  while(1)
  {
    wm8904ReadReg(0x70, &reg_data);
    if ((reg_data & 0x0001) == 0)
    {
      logPrintf("[  ] wm8904 Start-Up Done\n");
      logPrintf("[  ]        %ldms\n", millis()-pre_time);
      break;
    }

    if (millis()-pre_time >= 1000)
    {
      logPrintf("[E_] wm8904 Start-Up Timeout\n");
      ret = false;
      break;
    }
  }

  if (ret)
  {
    ret &= wm8904WriteReg(0x15, 0x0C02); // 16Khz  - Sample Rate :  
    ret &= wm8904WriteReg(0x19, 0x0002); // 16bits - Word Length
    ret &= wm8904WriteReg(0x21, 0x0000); // DAC Mute Off
  }

  return ret;
}

bool wm8904ReadReg(uint8_t addr, uint16_t *p_data)
{
  bool ret;
  uint8_t buf[2];


  ret = i2cReadBytes(i2c_ch, i2c_addr, addr, buf, 2, 10);
  if (ret)
  {
    *p_data = (buf[0]<<8) | (buf[1]<<0);
  }

  return ret;
}

bool wm8904WriteReg(uint8_t addr, uint16_t data)
{
  bool ret;
  uint8_t buf[2];


  buf[0] = data>>8;
  buf[1] = data>>0;
  ret = i2cWriteBytes(i2c_ch, i2c_addr, addr, buf, 2, 10);

  return ret;
}

#ifdef _USE_HW_CLI
void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "info") == true)
  {
    ret = true;
  }

  if(args->argc == 3 && args->isStr(0, "read") == true)
  {
    uint8_t addr;
    uint8_t length;

    addr = args->getData(1);
    length = args->getData(2);

    for (int i = 0; i < length; i++)
    {
      uint16_t reg_data;

      if (wm8904ReadReg(addr + i, &reg_data))
      {
        cliPrintf("0x%02X : 0x%04X\n", addr+i, reg_data);
      }
      else
      {
        cliPrintf("wm8904ReadReg() - Fail \n");
        break;
      }
    }
    ret = true;
  }

  if(args->argc == 3 && args->isStr(0, "write") == true)
  {
    uint8_t addr;
    uint16_t reg_data;

    addr = args->getData(1);
    reg_data = args->getData(2);


    if (wm8904WriteReg(addr, reg_data))
    {
      cliPrintf("0x%02X : 0x%04X\n", addr, reg_data);
    }
    else
    {
      cliPrintf("wm8904WriteReg() - Fail \n");
    }
    ret = true;
  }


  if (ret != true)
  {
    cliPrintf("wm8904 info\n");
    cliPrintf("wm8904 read  reg_addr length\n");
    cliPrintf("wm8904 write reg_addr data\n");
  }
}
#endif


#endif