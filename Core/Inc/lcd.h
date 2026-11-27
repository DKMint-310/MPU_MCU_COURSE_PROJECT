#ifndef LCD_H
#define LCD_H

#include "peripheral_i2c.h"

#define LCD_OHSTEM_ADDRESS  0x21U
#define LCD_PCF8574_ADDRESS 0x27U
#define LCD_GROVE_ADDRESS   0x3EU
#define LCD_COLUMNS         16U
#define LCD_ROWS            2U

/*
 * Supports both PCF8574 4-bit HD44780 backpacks (OhStem 0x21, standard 0x27/0x3F)
 * and Grove AiP31068 native I2C 16x2 LCDs (0x3E).
 */
bool lcd_configure(const peripheral_i2c_t *bus, uint8_t address);
/* Service power-up/retry in main context. True once the LCD is ready. */
bool lcd_init(void);
/* Zero-based row. Pad or truncate text to 16 characters. */
bool lcd_write_line(uint8_t row, const char *text);

#endif
