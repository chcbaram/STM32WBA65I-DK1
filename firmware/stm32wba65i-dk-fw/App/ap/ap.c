#include "ap.h"



void apInit(void)
{
  cliOpen(HW_UART_CH_CLI, 115200);
  cliLogo();

  for (int i = 0; i < 32; i += 2) 
  {
    lcdClearBuffer(black);
    lcdPrintfResize(0, 64 - 8 - i, green, 16, "  -- BARAM --");
    lcdDrawRect(0, 0, LCD_WIDTH, LCD_HEIGHT, white);
    lcdUpdateDraw();
  }  
  delay(500);
  lcdClear(black);  
}


void apMain(void)
{
  uint32_t pre_time;


  pre_time = millis();
  while(1)
  {  
    if (millis() - pre_time >= 500)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);
    }

    cliMain();

    if (uartAvailable(HW_UART_CH_USB))
    {
      uartPrintf(HW_UART_CH_USB, "rx cdc : 0x%02X\n", uartRead(HW_UART_CH_USB));
    }
  }
} 
