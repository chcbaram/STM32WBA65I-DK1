#include "hw.h"




bool hwInit(void)
{
  cliInit();
  ledInit();
  gpioInit();
  uartInit();
  for (int i=0; i<HW_UART_MAX_CH; i++)
  {
    uartOpen(i, 115200);
  }

  logPrintf("\r\n[ Firmware Begin... ]\r\n");
  logPrintf("Booting..Clock\t: %d Mhz\r\n", (int)HAL_RCC_GetSysClockFreq()/1000000);
  logPrintf("\n");

  i2cInit();
  adcInit();
  buttonInit();
  spiInit();
  spiFlashInit();
  saiInit();
  wm8904Init();
  lcdInit();
  lcdSetFps(20);

  cdcInit();
  usbInit();
  usbBegin(USB_CDC_MODE);

  return true;
}

void delay(uint32_t ms)
{
  HAL_Delay(ms);
}

uint32_t millis(void)
{
  return HAL_GetTick();
}
