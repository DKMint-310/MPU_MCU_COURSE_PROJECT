#ifndef LCD_H
#define LCD_H

#include "peripheral_i2c.h"

#define LCD_GROVE_ADDRESS 0x3EU
#define LCD_COLUMNS 16U
#define LCD_ROWS 2U

/* Grove 16x2 text LCD using command/data control bytes. */
bool lcd_configure(const peripheral_i2c_t *bus, uint8_t address);
/* Service power-up/retry in main context. True once the LCD is ready. */
bool lcd_init(void);
/* Zero-based row. Pad or truncate text to 16 characters. */
bool lcd_write_line(uint8_t row, const char *text);

#endif
