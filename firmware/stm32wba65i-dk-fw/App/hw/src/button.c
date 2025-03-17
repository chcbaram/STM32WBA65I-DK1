#include "button.h"


#ifdef _USE_HW_BUTTON
#include "adc.h"
#include "cli.h"

#define NAME_DEF(x) x, #x

enum
{
  TYPE_GPIO,
  TYPE_ADC,
};

typedef struct
{
  uint8_t       type;
  GPIO_TypeDef *port;
  uint32_t      pin;
  GPIO_PinState on_state;

  ButtonPinName_t pin_name;
  const char     *p_name;
} button_tbl_t;

static button_tbl_t button_tbl[BUTTON_MAX_CH] =
{
  {TYPE_ADC, NULL, 0, GPIO_PIN_RESET, NAME_DEF(BUTTON_UP)   },
  {TYPE_ADC, NULL, 0, GPIO_PIN_RESET, NAME_DEF(BUTTON_DOWN) },
  {TYPE_ADC, NULL, 0, GPIO_PIN_RESET, NAME_DEF(BUTTON_LEFT) },
  {TYPE_ADC, NULL, 0, GPIO_PIN_RESET, NAME_DEF(BUTTON_RIGHT)},
  {TYPE_ADC, NULL, 0, GPIO_PIN_RESET, NAME_DEF(BUTTON_ENTER)},
};


#ifdef _USE_HW_CLI
static void cliButton(cli_args_t *args);
#endif


bool buttonInit(void)
{
  bool ret = true;

#ifdef _USE_HW_CLI
  cliAdd("button", cliButton);
#endif

  return ret;
}

bool buttonGetPressed(uint8_t ch)
{
  bool ret = false;

  if (ch >= BUTTON_MAX_CH)
  {
    return false;
  }

  if (button_tbl[ch].type == TYPE_GPIO)
  {
    if (HAL_GPIO_ReadPin(button_tbl[ch].port, button_tbl[ch].pin) == button_tbl[ch].on_state)
    {
      ret = true;
    }
  }
  else
  {
    float margin = 0.1;
    float vol;

    vol = adcReadVoltage(BUTTON_ADC);

    switch (button_tbl[ch].pin_name)
    {
      case BUTTON_UP:
        if (vol >= 2.01 - margin && vol <= 2.01 + margin)
        {
          ret = true;
        }
        break;

      case BUTTON_DOWN:
        if (vol >= 1.32 - margin && vol <= 1.32 + margin)
        {
          ret = true;
        }
        break;

      case BUTTON_LEFT:
        if (vol >= 0.67 - margin && vol <= 0.67 + margin)
        {
          ret = true;
        }
        break;

      case BUTTON_RIGHT:
        if (vol >= 2.65 - margin && vol <= 2.65 + margin)
        {
          ret = true;
        }
        break;

      case BUTTON_ENTER:
        if (vol <= 0. + margin)
        {
          ret = true;
        }
        break;

      default:
        break;
    }
  }

  return ret;
}


#ifdef _USE_HW_CLI
void cliButton(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "show"))
  {
    while (cliKeepLoop())
    {
      for (int i = 0; i < BUTTON_MAX_CH; i++)
      {
        cliPrintf("%d", buttonGetPressed(i));
      }
      cliPrintf("\n");

      delay(100);
    }

    ret = true;
  }


  if (ret != true)
  {
    cliPrintf("button show\n");
  }
}
#endif


#endif
