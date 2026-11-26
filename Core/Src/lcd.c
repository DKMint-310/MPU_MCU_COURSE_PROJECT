#include "lcd.h"

#include <stddef.h>
#include <string.h>

static peripheral_i2c_t transport;
static uint8_t lcd_address;
static uint8_t init_step;
static uint32_t wait_started;
static uint32_t wait_ms;
static char displayed[LCD_ROWS][LCD_COLUMNS];
static bool row_valid[LCD_ROWS];

static const uint8_t init_commands[] = {0x28, 0x28, 0x28, 0x28, 0x0C, 0x01, 0x06};
static const uint8_t init_wait_ms[] = {5, 1, 1, 1, 1, 2, 1};

static void wait_for(uint32_t delay)
{
    wait_started = transport.now_ms();
    wait_ms = delay;
}

static void reset_display(uint32_t delay)
{
    init_step = 0;
    memset(row_valid, 0, sizeof row_valid);
    wait_for(delay);
}

bool lcd_configure(const peripheral_i2c_t *bus, uint8_t address)
{
    if (bus == NULL || bus->write == NULL || bus->now_ms == NULL ||
        address < 0x08U || address > 0x77U || address == 0x38U) {
        return false;
    }

    transport = *bus;
    lcd_address = address;
    reset_display(50);
    return true;
}

bool lcd_init(void)
{
    if (transport.now_ms == NULL || transport.now_ms() - wait_started < wait_ms) {
        return false;
    }
    if (init_step == sizeof init_commands) {
        return true;
    }

    const uint8_t command[] = {0x80, init_commands[init_step]};
    if (!transport.write(transport.context, lcd_address, command, sizeof command)) {
        reset_display(500);
        return false;
    }
    wait_for(init_wait_ms[init_step]);
    ++init_step;
    return false;
}

bool lcd_write_line(uint8_t row, const char *text)
{
    if (row >= LCD_ROWS || text == NULL || init_step != sizeof init_commands ||
        transport.now_ms() - wait_started < wait_ms) {
        return false;
    }

    /* Co=1 for the cursor command, then Co=0 and RS=1 for the text. */
    uint8_t frame[3 + LCD_COLUMNS] = {0x80, row == 0 ? 0x80 : 0xC0, 0x40};
    memset(frame + 3, ' ', LCD_COLUMNS);
    for (uint8_t col = 0; col < LCD_COLUMNS && text[col] != '\0'; ++col) {
        frame[3 + col] = (uint8_t)text[col];
    }
    if (row_valid[row] && memcmp(displayed[row], frame + 3, LCD_COLUMNS) == 0) {
        return true;
    }

    if (!transport.write(transport.context, lcd_address, frame, sizeof frame)) {
        reset_display(500);
        return false;
    }
    memcpy(displayed[row], frame + 3, LCD_COLUMNS);
    row_valid[row] = true;
    return true;
}
