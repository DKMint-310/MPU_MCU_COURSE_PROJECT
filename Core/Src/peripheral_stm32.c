#include "peripheral_stm32.h"

#include <stddef.h>
#include <string.h>

static peripheral_pin_t input_pins[BUTTON_COUNT];
static peripheral_pin_t light_pins[4];

static bool valid_pins(const peripheral_pin_t *pins, size_t count)
{
    if (pins == NULL) {
        return false;
    }
    for (size_t i = 0; i < count; ++i) {
        uint16_t pin = pins[i].pin;
        if (pins[i].port == NULL || pin == 0U || (pin & (pin - 1U)) != 0U) {
            return false;
        }
        for (size_t j = 0; j < i; ++j) {
            if (pins[i].port == pins[j].port && pin == pins[j].pin) {
                return false;
            }
        }
    }
    return true;
}

static bool read_gpio(void *context, uint8_t id)
{
    (void)context;
    peripheral_pin_t pin = input_pins[id];
    return HAL_GPIO_ReadPin(pin.port, pin.pin) == GPIO_PIN_SET;
}

static void write_gpio(void *context, uint8_t road, uint8_t pin_index, bool high)
{
    (void)context;
    peripheral_pin_t pin = light_pins[2U * road + pin_index];
    HAL_GPIO_WritePin(pin.port, pin.pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool peripheral_stm32_bind_buttons(const peripheral_pin_t pins[BUTTON_COUNT],
                                   const bool active_high[BUTTON_COUNT])
{
    if (!valid_pins(pins, BUTTON_COUNT) || !button_init(read_gpio, NULL, active_high)) {
        return false;
    }
    memcpy(input_pins, pins, sizeof input_pins);
    return true;
}

bool peripheral_stm32_bind_lights(const peripheral_pin_t pins[4],
                                  const traffic_codes_t codes[2])
{
    if (!valid_pins(pins, 4) || !traffic_light_init(write_gpio, NULL, codes)) {
        return false;
    }
    memcpy(light_pins, pins, sizeof light_pins);
    return true;
}

#ifdef HAL_I2C_MODULE_ENABLED
enum { I2C_TRANSFER_TIMEOUT_MS = 3 };

static bool i2c_available(I2C_HandleTypeDef *handle)
{
    /* Skip HAL's fixed BUSY wait when the bus is already stuck. */
    return handle != NULL && HAL_I2C_GetState(handle) == HAL_I2C_STATE_READY &&
           !__HAL_I2C_GET_FLAG(handle, I2C_FLAG_BUSY);
}

static bool i2c_write(void *context, uint8_t address, const uint8_t *data, uint16_t size)
{
    I2C_HandleTypeDef *handle = context;
    return i2c_available(handle) &&
           HAL_I2C_Master_Transmit(handle, (uint16_t)(address << 1),
                                   (uint8_t *)data, size, I2C_TRANSFER_TIMEOUT_MS) == HAL_OK;
}

static bool i2c_read(void *context, uint8_t address, uint8_t *data, uint16_t size)
{
    I2C_HandleTypeDef *handle = context;
    return i2c_available(handle) &&
           HAL_I2C_Master_Receive(handle, (uint16_t)(address << 1),
                                  data, size, I2C_TRANSFER_TIMEOUT_MS) == HAL_OK;
}

peripheral_i2c_t peripheral_stm32_i2c(I2C_HandleTypeDef *handle)
{
    return (peripheral_i2c_t){
        .context = handle,
        .write = i2c_write,
        .read = i2c_read,
        .now_ms = HAL_GetTick
    };
}
#endif
