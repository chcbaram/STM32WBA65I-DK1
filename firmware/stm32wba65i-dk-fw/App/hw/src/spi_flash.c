#include "spi_flash.h"



#ifdef _USE_HW_SPI_FLASH
#include "qspi/mp95p32.h"
#include "spi.h"
#include "gpio.h"
#include "cli.h"

#define SPI_CS_L()    gpioPinWrite(SPI_FLASH_NSS, _DEF_LOW)
#define SPI_CS_H()    gpioPinWrite(SPI_FLASH_NSS, _DEF_HIGH)


static bool is_init = false;
static uint8_t spi_ch = _DEF_SPI1;




static bool spiFlashTransfer(uint8_t *tx_buf, uint8_t *rx_buf, uint32_t length, uint32_t timeout);
static bool spiFlashGetID(uint8_t *p_id_tbl, uint32_t lenght);
static bool spiFlashWriteEnable(void);
static bool spiFlashWaitBusy(uint32_t timeout);


#ifdef _USE_HW_CLI
static void cliCmd(cli_args_t *args);
#endif



bool spiFlashInit(void)
{
  bool ret = true;
  uint8_t id_tbl[4];

  ret = spiFlashReset();

  if (spiFlashGetID(id_tbl, 4) == true)
  {
    if (id_tbl[0] == 0x20 && id_tbl[1] == 0x00 && id_tbl[2] == 0x16)
    {
      logPrintf("[OK] spiFlashInit()\n");
      logPrintf("     MP95P32 Found\r\n");
      ret = true;
    }
    else
    {
      logPrintf("[NG] spiFlashInit()\n");
      logPrintf("     MP95P32 Not Found %X %X %X %X\r\n", id_tbl[0], id_tbl[1], id_tbl[2], id_tbl[3]);      
      ret = false;
    }
  }
  else
  {
    logPrintf("spiFlash            \t: Fail\r\n");
    ret = false;
  }

  is_init = ret;

#ifdef _USE_HW_CLI
  cliAdd("spiFlash", cliCmd);
#endif

  return ret;
}

bool spiFlashIsInit(void)
{
  return is_init;
}

bool spiFlashReset(void)
{
  bool ret = true;


  if (spiBegin(spi_ch) == false)
  {
    return false;
  }

  uint8_t tx_buf[4];


  //-- Reset Memory
  //
  tx_buf[0] = RESET_ENABLE_CMD;

  SPI_CS_L();
  ret &= spiFlashTransfer(tx_buf, NULL, 1, 10);
  SPI_CS_H();

  tx_buf[0] = RESET_MEMORY_CMD;
  SPI_CS_L();
  ret &= spiFlashTransfer(tx_buf, NULL, 1, 10);
  SPI_CS_H();

  delay(10);
  if (ret != true) return false;



  return ret;
}

bool spiFlashRead(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  bool ret = true;
  uint8_t tx_buf[5];

  tx_buf[0] = FAST_READ_CMD;
  tx_buf[1] = addr >> 16;
  tx_buf[2] = addr >> 8;
  tx_buf[3] = addr >> 0;
  tx_buf[4] = 0; // Dummy

  SPI_CS_L();
  ret &= spiFlashTransfer(tx_buf, NULL, 5, 10);
  if (ret == true)
  {
    ret &= spiFlashTransfer(NULL, p_data, length, 500);
  }
  SPI_CS_H();

  #ifdef _USE_HW_CACHE
  SCB_InvalidateDCache_by_Addr ((uint32_t *)p_data, length);
  #endif

  return ret;
}

bool spiFlashWrite(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  bool ret = true;
  uint8_t tx_buf[4];
  uint32_t end_addr, current_size, current_addr;


  if (addr+length > spiFlashGetLength())
  {
    return false;
  }


  /* Calculation of the size between the write address and the end of the page */
  current_size = MP95P32_PAGE_SIZE - (addr % MP95P32_PAGE_SIZE);

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > length)
  {
    current_size = length;
  }

  /* Initialize the adress variables */
  current_addr = addr;
  end_addr = addr + length;


  /* Perform the write page by page */
  do
  {
    //-- Write Enable
    //
    if (spiFlashWriteEnable() == false)
    {
      return false;
    }

    //-- Write Page
    //
    SPI_CS_L();
    tx_buf[0] = PAGE_PROG_CMD;
    tx_buf[1] = current_addr >> 16;
    tx_buf[2] = current_addr >> 8;
    tx_buf[3] = current_addr >> 0;
    ret &= spiFlashTransfer(tx_buf, NULL, 4, 10);

    ret &= spiFlashTransfer(p_data, NULL, current_size, 10);
    SPI_CS_H();


    ret &= spiFlashWaitBusy(2000);
    if (ret == false)
    {
      break;
    }

    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    p_data += current_size;
    current_size = ((current_addr + MP95P32_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : MP95P32_PAGE_SIZE;
  } while (current_addr < end_addr);


  return ret;
}

bool spiFlashErase(uint32_t addr, uint32_t length)
{
  bool ret = true;
  uint32_t flash_length;
  uint32_t block_size;
  uint32_t block_begin;
  uint32_t block_end;
  uint32_t i;



  flash_length = MP95P32_FLASH_SIZE;
  block_size   = MP95P32_SECTOR_SIZE;


  if ((addr > flash_length) || ((addr+length) > flash_length))
  {
    return false;
  }
  if (length == 0)
  {
    return false;
  }


  block_begin = addr / block_size;
  block_end   = (addr + length - 1) / block_size;


  for (i=block_begin; i<=block_end; i++)
  {
    ret = spiFlashEraseBlock(block_size*i);
    if (ret == false)
    {
      break;
    }
  }

  return ret;
}

bool spiFlashEraseBlock(uint32_t block_addr)
{
  bool ret = true;
  uint8_t tx_buf[4];


  //-- Write Enable
  //
  if (spiFlashWriteEnable() == false)
  {
    return false;
  }


  //-- Erase Sector
  //
  SPI_CS_L();
  tx_buf[0] = SECTOR_ERASE_CMD;
  tx_buf[1] = block_addr >> 16;
  tx_buf[2] = block_addr >> 8;
  tx_buf[3] = block_addr >> 0;
  ret &= spiFlashTransfer(tx_buf, NULL, 4, 10);
  SPI_CS_H();

  ret &= spiFlashWaitBusy(MP95P32_SECTOR_ERASE_MAX_TIME);

  return ret;
}

bool spiFlashEraseSector(uint32_t sector_addr)
{
  bool ret = true;
  uint8_t tx_buf[4];


  //-- Write Enable
  //
  if (spiFlashWriteEnable() == false)
  {
    return false;
  }


  //-- Erase Sector
  //
  SPI_CS_L();
  tx_buf[0] = SUBSECTOR_ERASE_CMD;
  tx_buf[1] = sector_addr >> 16;
  tx_buf[2] = sector_addr >> 8;
  tx_buf[3] = sector_addr >> 0;
  ret &= spiFlashTransfer(tx_buf, NULL, 4, 10);
  SPI_CS_H();

  ret &= spiFlashWaitBusy(MP95P32_SUBSECTOR_ERASE_MAX_TIME);

  return ret;
}

uint32_t spiFlashGetAddr(void)
{
  return HW_SPI_FLASH_ADDR;
}

uint32_t spiFlashGetLength(void)
{
  return MP95P32_FLASH_SIZE;
}

bool spiFlashEraseChip(void);
bool spiFlashGetStatus(void);
bool spiFlashGetInfo(spi_flash_info_t* p_info);

bool spiFlashGetID(uint8_t *p_id_tbl, uint32_t length)
{
  bool ret = true;

  uint8_t tx_buf[9] = {0, };
  uint8_t rx_buf[9] = {0, };

  //-- Read ID
  //
  tx_buf[0] = READ_ID_CMD;
  SPI_CS_L();
  ret &= spiFlashTransfer(tx_buf, rx_buf, 9, 10);
  SPI_CS_H();
  if (ret == true)
  {
    for (int i=0; i<(int)length; i++)
    {
      p_id_tbl[i] = rx_buf[4 + i];
    }
  }
  return ret;
}

bool spiFlashWriteEnable(void)
{
  bool ret = true;
  uint8_t tx_buf[4];
  uint8_t rx_buf[4];
  uint32_t pre_time;


  //-- Write Enable
  //
  SPI_CS_L();
  tx_buf[0] = WRITE_ENABLE_CMD;
  ret &= spiFlashTransfer(tx_buf, NULL, 1, 10);
  SPI_CS_H();

  pre_time = millis();
  while(1)
  {
    SPI_CS_L();
    tx_buf[0] = READ_STATUS_REG_CMD;
    ret &= spiFlashTransfer(tx_buf, rx_buf, 2, 10);
    SPI_CS_H();

    if (ret == true)
    {
      if (rx_buf[1] & MP95P32_SR_WREN)
      {
        break;
      }
    }
    if (millis()-pre_time >= 100)
    {
      ret = false;
      break;
    }
  }

  return ret;
}

bool spiFlashWaitBusy(uint32_t timeout)
{
  bool ret = true;
  uint32_t pre_time;
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];


  pre_time = millis();
  while(1)
  {
    if (millis()-pre_time >= timeout)
    {
      ret = false;
      break;
    }

    SPI_CS_L();
    tx_buf[0] = READ_STATUS_REG_CMD;
    ret &= spiFlashTransfer(tx_buf, rx_buf, 2, 10);
    SPI_CS_H();

    if (ret == true)
    {
      uint8_t busy_bit;

      busy_bit = rx_buf[1] & MP95P32_SR_WIP;
      if (busy_bit == 0)
      {
        break;
      }
    }
  }


  return ret;
}

bool spiFlashTransfer(uint8_t *tx_buf, uint8_t *rx_buf, uint32_t length, uint32_t timeout)
{
  bool ret = true;

  ret = spiTransferDMA(spi_ch, tx_buf, rx_buf, length, timeout);  
  // ret = spiTransfer(spi_ch, tx_buf, rx_buf, length, timeout);  

  return ret;
}



#ifdef _USE_HW_CLI
void cliCmd(cli_args_t *args)
{
  bool ret = false;
  uint32_t i;
  uint32_t addr;
  uint32_t length;
  uint8_t  data;
  uint32_t pre_time;
  bool flash_ret;


  if (args->argc == 1 && args->isStr(0, "info"))
  {
    cliPrintf("flash addr  : 0x%X\n", 0x0000000);
    ret = true;
  }
  
  if(args->argc == 1 && args->isStr(0, "test"))
  {
    uint8_t rx_buf[256];

    for (int i=0; i<100; i++)
    {
      if (spiFlashRead(0x1000*i, rx_buf, 256))
      {
        cliPrintf("%d : OK\n", i);
      }
      else
      {
        cliPrintf("%d : FAIL\n", i);
        break;
      }
    }
    ret = true;
  }    
  
  if (args->argc == 3 && args->isStr(0, "read"))
  {
    addr   = (uint32_t)args->getData(1);
    length = (uint32_t)args->getData(2);

    for (i=0; i<length; i++)
    {
      flash_ret = spiFlashRead(addr+i, &data, 1);

      if (flash_ret == true)
      {
        cliPrintf( "addr : 0x%X\t 0x%02X\n", addr+i, data);
      }
      else
      {
        cliPrintf( "addr : 0x%X\t Fail\n", addr+i);
      }
    }
    ret = true;
  }
  
  if(args->argc == 3 && args->isStr(0, "erase") == true)
  {
    addr   = (uint32_t)args->getData(1);
    length = (uint32_t)args->getData(2);

    pre_time = millis();
    flash_ret = spiFlashErase(addr, length);

    cliPrintf( "addr : 0x%X\t len : %d %d ms\n", addr, length, (millis()-pre_time));
    if (flash_ret)
    {
      cliPrintf("OK\n");
    }
    else
    {
      cliPrintf("FAIL\n");
    }
    ret = true;
  }

  if(args->argc == 3 && args->isStr(0, "write"))
  {
    uint32_t flash_data;

    addr = (uint32_t)args->getData(1);
    flash_data = (uint32_t )args->getData(2);

    pre_time = millis();
    flash_ret = spiFlashWrite(addr, (uint8_t *)&flash_data, 4);

    cliPrintf( "addr : 0x%X\t 0x%X %dms\n", addr, flash_data, millis()-pre_time);
    if (flash_ret)
    {
      cliPrintf("OK\n");
    }
    else
    {
      cliPrintf("FAIL\n");
    }
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "speed-test") == true)
  {
    uint32_t buf[512/4];
    uint32_t cnt;
    uint32_t pre_time;
    uint32_t exe_time;


    cnt = 1024*1024 / 512;
    pre_time = millis();
    for (int i=0; i<(int)cnt; i++)
    {
      if (spiFlashRead(i*512, (uint8_t *)buf, 512) == false)
      {
        cliPrintf("spiFlashRead() Fail:%d\n", i);
        break;
      }
    }
    exe_time = millis()-pre_time;
    if (exe_time > 0)
    {
      cliPrintf("%d KB/sec\n", 1024 * 1000 / exe_time);
    }
    ret = true;
  }

  if (ret == false)
  {
    cliPrintf( "spiFlash info\n");
    cliPrintf( "spiFlash test\n");
    cliPrintf( "spiFlash speed-test\n");
    cliPrintf( "spiFlash read  [addr] [length]\n");
    cliPrintf( "spiFlash erase [addr] [length]\n");
    cliPrintf( "spiFlash write [addr] [data]\n");
  }

}
#endif

#endif


