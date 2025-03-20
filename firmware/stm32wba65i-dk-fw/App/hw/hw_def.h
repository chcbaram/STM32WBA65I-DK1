#ifndef HW_DEF_H_
#define HW_DEF_H_


#include "main.h"
#include "def.h"


#define _USE_HW_SAI
#define _USE_HW_BUZZER
#define _USE_HW_WM8904


#define _USE_HW_LED
#define      HW_LED_MAX_CH          2

#define _USE_HW_UART
#define      HW_UART_MAX_CH         2
#define      HW_UART_CH_SWD         _DEF_UART1
#define      HW_UART_CH_CLI         _DEF_UART1
#define      HW_UART_CH_USB         _DEF_UART2

#define _USE_HW_CLI
#define      HW_CLI_CMD_LIST_MAX    32
#define      HW_CLI_CMD_NAME_MAX    16
#define      HW_CLI_LINE_HIS_MAX    8
#define      HW_CLI_LINE_BUF_MAX    64

#define _USE_HW_USB
#define _USE_HW_CDC
#define      HW_USB_LOG             0
#define      HW_USB_CDC             1

#define _USE_HW_GPIO
#define      HW_GPIO_MAX_CH         GPIO_PIN_MAX

#define _USE_HW_SPI
#define      HW_SPI_MAX_CH          1

#define _USE_HW_SPI_FLASH
#define      HW_SPI_FLASH_ADDR      0x91000000

#define _USE_HW_LCD
#define _USE_HW_SSD1315
#define      HW_LCD_WIDTH           128
#define      HW_LCD_HEIGHT          64

#define _USE_HW_ADC                 
#define      HW_ADC_MAX_CH          ADC_PIN_MAX

#define _USE_HW_BUTTON
#define      HW_BUTTON_MAX_CH       BUTTON_PIN_MAX

#define _USE_HW_I2C
#define      HW_I2C_MAX_CH          1

#define _USE_HW_MIXER
#define      HW_MIXER_MAX_CH        4
#define      HW_MIXER_MAX_BUF_LEN   (48*2*4*4) // 48Khz * Stereo * 4ms * 2



typedef enum
{
  OLED_CS,
  OLED_RST,
  OLED_DC,
  SPI_FLASH_NSS,
  GPIO_PIN_MAX,
} GpioPinName_t;

typedef enum
{
  BUTTON_ADC,
  ADC_PIN_MAX
} AdcPinName_t;

typedef enum
{
  BUTTON_UP,
  BUTTON_DOWN,
  BUTTON_LEFT,
  BUTTON_RIGHT,
  BUTTON_ENTER,
  BUTTON_PIN_MAX
} ButtonPinName_t;


#define logPrintf printf

void     delay(uint32_t ms);
uint32_t millis(void);


#endif
