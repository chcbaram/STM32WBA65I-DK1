#include "lcd/ssd1315.h"
#include "lcd/ssd1315_regs.h"


#ifdef _USE_HW_SSD1315
#include "spi.h"
#include "gpio.h"

#define SSD1315_WIDTH       HW_LCD_WIDTH
#define SSD1315_HEIGHT      HW_LCD_HEIGHT


static uint8_t spi_ch  = _DEF_SPI1;
static void (*frameCallBack)(void) = NULL;


static bool ssd1315Reset(void);
static void ssd1315SetWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1);
static uint16_t ssd1315GetWidth(void);
static uint16_t ssd1315GetHeight(void);
static bool ssd1315SendBuffer(uint8_t *p_data, uint32_t length, uint32_t timeout_ms);
static bool ssd1315SetCallBack(void (*p_func)(void));
static void ssd1315Fill(uint16_t color);
static bool ssd1315UpdateDraw(void);
static void ssd1315DrawPixel(uint8_t x, uint8_t y, uint16_t color);


static uint8_t ssd1315_buffer[SSD1315_WIDTH * SSD1315_HEIGHT / 8];



bool ssd1315Init(void)
{
  bool ret;

  ret = ssd1315Reset();

  return ret;
}

bool ssd1315InitDriver(lcd_driver_t *p_driver)
{
  p_driver->init        = ssd1315Init;
  p_driver->reset       = ssd1315Reset;
  p_driver->setWindow   = ssd1315SetWindow;
  p_driver->getWidth    = ssd1315GetWidth;
  p_driver->getHeight   = ssd1315GetHeight;
  p_driver->setCallBack = ssd1315SetCallBack;
  p_driver->sendBuffer  = ssd1315SendBuffer;
  return true;
}

bool ssd1315WriteCmd(uint8_t cmd_data)
{
  bool ret;

  gpioPinWrite(OLED_DC, _DEF_LOW);
  gpioPinWrite(OLED_CS, _DEF_LOW);
  ret = spiTransfer(spi_ch, &cmd_data, NULL, 1, 50);
  gpioPinWrite(OLED_CS, _DEF_HIGH);  

  return ret;
}

bool ssd1315Reset(void)
{
  bool ret;

  if (spiIsBegin(spi_ch))
  {
    ret = true;
  }
  else
  {
    ret = spiBegin(spi_ch);
  }

  if (ret != true)
  {
    return false;
  }

  /* Init LCD */
  ssd1315WriteCmd(0xAE); //display off
  ssd1315WriteCmd(0x20); //Set Memory Addressing Mode
  ssd1315WriteCmd(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
  ssd1315WriteCmd(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
  ssd1315WriteCmd(0xC8); //Set COM Output Scan Direction
  ssd1315WriteCmd(0x00); //---set low column address
  ssd1315WriteCmd(0x10); //---set high column address
  ssd1315WriteCmd(0x40); //--set start line address
  ssd1315WriteCmd(0x81); //--set contrast control register
  ssd1315WriteCmd(0xFF);
  ssd1315WriteCmd(0xA1); //--set segment re-map 0 to 127
  ssd1315WriteCmd(0xA6); //--set normal display
  ssd1315WriteCmd(0xA8); //--set multiplex ratio(1 to 64)
  ssd1315WriteCmd(SSD1315_HEIGHT-1); // 0x3F
  ssd1315WriteCmd(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
  ssd1315WriteCmd(0xD3); //-set display offset
  ssd1315WriteCmd(0x00); //-not offset
  ssd1315WriteCmd(0xD5); //--set display clock divide ratio/oscillator frequency
  //ssd1315WriteCmd(0xF0); //--set divide ratio
  ssd1315WriteCmd(0x80); //--set divide ratio
  ssd1315WriteCmd(0xD9); //--set pre-charge period
  ssd1315WriteCmd(0x22); //

  #if SSD1315_HEIGHT == 64
  ssd1315WriteCmd(0xDA); //--set com pins hardware configuration
  ssd1315WriteCmd(0x12);
  #else
  ssd1315WriteCmd(0xDA); //--set com pins hardware configuration
  ssd1315WriteCmd(0x02);
  #endif
  ssd1315WriteCmd(0xDB); //--set vcomh
  ssd1315WriteCmd(0x20); //0x20,0.77xVcc
  ssd1315WriteCmd(0x8D); //--set DC-DC enable
  ssd1315WriteCmd(0x14); //
  ssd1315WriteCmd(0xAF); //--turn on SSD1315 panel

  ssd1315Fill(black);
  ssd1315UpdateDraw();

  return true;
}

void ssd1315SetWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  (void)x0;
  (void)y0;
  (void)x1;
  (void)y1;

}

uint16_t ssd1315GetWidth(void)
{
  return LCD_WIDTH;
}

uint16_t ssd1315GetHeight(void)
{
  return LCD_HEIGHT;
}

bool ssd1315SendBuffer(uint8_t *p_data, uint32_t length, uint32_t timeout_ms)
{
  uint16_t *p_buf = (uint16_t *)p_data;
  (void)length;
  (void)timeout_ms;


  for (int y=0; y<SSD1315_HEIGHT; y++)
  {
    for (int x=0; x<SSD1315_WIDTH; x++)
    {
      ssd1315DrawPixel(x, y, p_buf[y*LCD_WIDTH + x]);
    }
  }

  ssd1315UpdateDraw();

  if (frameCallBack != NULL)
  {
    frameCallBack();
  }
  return true;
}

bool ssd1315SetCallBack(void (*p_func)(void))
{
  frameCallBack = p_func;

  return true;
}

void ssd1315Fill(uint16_t color)
{
  uint32_t i;

  for(i = 0; i < sizeof(ssd1315_buffer); i++)
  {
    ssd1315_buffer[i] = (color > 0) ? 0xFF : 0x00;
  }
}

bool ssd1315UpdateDraw(void)
{
  uint8_t i;
  bool ret;

  for (i = 0; i < SSD1315_HEIGHT/8; i++)
  {
    ssd1315WriteCmd(0xB0 + i);
    ssd1315WriteCmd(0x00);
    ssd1315WriteCmd(0x10);

    gpioPinWrite(OLED_DC, _DEF_HIGH);
    gpioPinWrite(OLED_CS, _DEF_LOW);
    ret = spiTransfer(spi_ch, &ssd1315_buffer[SSD1315_WIDTH * i], NULL, SSD1315_WIDTH, 100);
    gpioPinWrite(OLED_CS, _DEF_HIGH);   
    if (ret != true)
    {
      break;
    }    
  }

  return true;
}

void ssd1315DrawPixel(uint8_t x, uint8_t y, uint16_t color)
{
  if (x >= SSD1315_WIDTH || y >= SSD1315_HEIGHT)
  {
    return;
  }


  if (color > 0)
  {
    ssd1315_buffer[x + (y / 8) * SSD1315_WIDTH] |= 1 << (y % 8);
  }
  else
  {
    ssd1315_buffer[x + (y / 8) * SSD1315_WIDTH] &= ~(1 << (y % 8));
  }
}

#endif