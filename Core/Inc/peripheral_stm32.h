#ifndef PERIPHERAL_STM32_H
#define PERIPHERAL_STM32_H

#include "stm32f1xx_hal.h"

#include "button.h"
#include "peripheral_i2c.h"
#include "traffic_light.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} peripheral_pin_t;

/* GPIO direction, pull and initial red levels are configured by CubeMX. */
bool peripheral_stm32_bind_buttons(const peripheral_pin_t pins[BUTTON_COUNT],
                                   const bool active_high[BUTTON_COUNT]);
/* Pin order is road 1 A/B followed by road 2 A/B. */
bool peripheral_stm32_bind_lights(const peripheral_pin_t pins[4],
                                  const traffic_codes_t codes[2]);

#ifdef HAL_I2C_MODULE_ENABLED
/* Use a single-master bus at 100 kHz. CubeMX provides the initialized handle. */
peripheral_i2c_t peripheral_stm32_i2c(I2C_HandleTypeDef *handle);
/* Scans I2C bus for a connected LCD (skipping DHT20 at 0x38), or returns fallback_addr. */
uint8_t peripheral_stm32_scan_lcd(I2C_HandleTypeDef *handle, uint8_t fallback_addr);
#endif

#endif
