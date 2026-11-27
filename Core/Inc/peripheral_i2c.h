#ifndef PERIPHERAL_I2C_H
#define PERIPHERAL_I2C_H

#include <stdbool.h>
#include <stdint.h>

/* Main-context transport. Callbacks complete the transfer before returning
 * and must use a finite timeout. All addresses are unshifted 7-bit values. */
typedef struct {
    void *context;
    bool (*write)(void *context, uint8_t address, const uint8_t *data, uint16_t size);
    bool (*read)(void *context, uint8_t address, uint8_t *data, uint16_t size);
    uint32_t (*now_ms)(void);
} peripheral_i2c_t;

#endif
