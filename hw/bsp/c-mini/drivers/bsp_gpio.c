#include "bsp_gpio.h"

#include "bsp_pwm.h"
#include "main.h"

typedef struct {
  uint16_t pin;
  GPIO_TypeDef *gpio;
} bsp_gpio_map_t;

static const bsp_gpio_map_t bsp_gpio_map[BSP_GPIO_NUM] = {
    {ACCL_CS_Pin, ACCL_CS_GPIO_Port},   {GYRO_CS_Pin, GYRO_CS_GPIO_Port},
    {ACCL_INT_Pin, ACCL_INT_GPIO_Port}, {GYRO_INT_Pin, GYRO_INT_GPIO_Port},
    {LED_Pin, LED_GPIO_Port},
};

static bsp_callback_t callback_list[16];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  for (uint8_t i = 0; i < 16; i++) {
    if (GPIO_Pin & (1 << i)) {
      bsp_callback_t cb = callback_list[i];

      if (cb.fn) {
        cb.fn(cb.arg);
      }
    }
  }
}

bsp_status_t bsp_gpio_register_callback(bsp_gpio_t gpio,
                                        void (*callback)(void *),
                                        void *callback_arg) {
  assert_param(callback);

  uint16_t pin = bsp_gpio_map[gpio].pin;

  for (uint8_t i = 0; i < 16; i++) {
    if (pin & (1 << i)) {
      callback_list[i].fn = callback;
      callback_list[i].arg = callback_arg;
      return BSP_OK;
    }
  }
  return BSP_ERR;
}

bsp_status_t bsp_gpio_enable_irq(bsp_gpio_t gpio) {
  uint16_t pin = bsp_gpio_map[gpio].pin;

  switch (pin) {
    case ACCL_INT_Pin:
      HAL_NVIC_EnableIRQ(ACCL_INT_EXTI_IRQn);
      break;

    case GYRO_INT_Pin:
      HAL_NVIC_EnableIRQ(GYRO_INT_EXTI_IRQn);
      break;
      /*
      case XXX_Pin:
              HAL_NVIC_EnableIRQ(XXX_IRQn);
              break;
      */

    default:
      return BSP_ERR;
  }
  return BSP_OK;
}

bsp_status_t bsp_gpio_disable_irq(bsp_gpio_t gpio) {
  uint16_t pin = bsp_gpio_map[gpio].pin;

  switch (pin) {
    case ACCL_INT_Pin:
      HAL_NVIC_DisableIRQ(ACCL_INT_EXTI_IRQn);
      break;

    case GYRO_INT_Pin:
      HAL_NVIC_DisableIRQ(GYRO_INT_EXTI_IRQn);
      break;
      /*
      case XXX_Pin:
              HAL_NVIC_EnableIRQ(XXX_IRQn);
              break;
      */

    default:
      return BSP_ERR;
  }
  return BSP_OK;
}

inline bsp_status_t bsp_gpio_write_pin(bsp_gpio_t gpio, bool value) {
  if (gpio == BSP_GPIO_SWITCH) {
    bsp_pwm_set_comp(BSP_PWM_SERVO_B, 1.0f);
    if (value) {
      bsp_pwm_start(BSP_PWM_SERVO_B);
    } else {
      bsp_pwm_stop(BSP_PWM_SERVO_B);
    }

    return BSP_OK;
  }

  HAL_GPIO_WritePin(bsp_gpio_map[gpio].gpio, bsp_gpio_map[gpio].pin, value);
  return BSP_OK;
}

inline bool bsp_gpio_read_pin(bsp_gpio_t gpio) {
  return HAL_GPIO_ReadPin(bsp_gpio_map[gpio].gpio, bsp_gpio_map[gpio].pin);
}
