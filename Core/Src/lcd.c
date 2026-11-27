#include "lcd.h"

#include <stddef.h>
#include <string.h>

/* PCF8574 pin mapping: P7..P4 = D7..D4, P3 = BL (0x08), P2 = EN (0x04), P0 = RS (0x01) */
#define PCF_BL 0x08U
#define PCF_EN 0x04U
#define PCF_RS 0x01U

static peripheral_i2c_t transport;
static uint8_t lcd_address;
static uint8_t init_step;
static uint32_t wait_started;
static uint32_t wait_ms;
static char displayed[LCD_ROWS][LCD_COLUMNS];
static bool row_valid[LCD_ROWS];

/* Grove AiP31068 (0x3E) init sequence */
static const uint8_t grove_init_cmds[] = {0x28, 0x28, 0x28, 0x28, 0x0C, 0x01, 0x06};
static const uint8_t grove_init_waits[] = {5, 1, 1, 1, 1, 2, 1};

/* PCF8574 HD44780 4-bit init: steps 0..3 send upper nibble only; steps 4..8 send full byte */
static const uint8_t pcf_init_cmds[] = {0x30, 0x30, 0x30, 0x20, 0x28, 0x08, 0x01, 0x06, 0x0C};
static const uint8_t pcf_init_waits[] = {5, 1, 1, 1, 1, 1, 2, 1, 1};

static bool is_grove_aip31068(void)
{
    return lcd_address == LCD_GROVE_ADDRESS;
}

static uint8_t total_init_steps(void)
{
    return is_grove_aip31068() ? (uint8_t)sizeof(grove_init_cmds)
                               : (uint8_t)sizeof(pcf_init_cmds);
}

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

static bool pcf_write_nibble(uint8_t upper_nibble, uint8_t rs)
{
    uint8_t hi = (uint8_t)((upper_nibble & 0xF0U) | PCF_BL | rs);
    uint8_t pulse[2] = {(uint8_t)(hi | PCF_EN), hi};
    return transport.write(transport.context, lcd_address, pulse, sizeof pulse);
}

static bool pcf_write_byte(uint8_t value, uint8_t rs)
{
    uint8_t hi = (uint8_t)((value & 0xF0U) | PCF_BL | rs);
    uint8_t lo = (uint8_t)(((value << 4) & 0xF0U) | PCF_BL | rs);
    uint8_t frame[4] = {
        (uint8_t)(hi | PCF_EN), hi,
        (uint8_t)(lo | PCF_EN), lo
    };
    return transport.write(transport.context, lcd_address, frame, sizeof frame);
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
    uint8_t needed = total_init_steps();

    if (transport.now_ms == NULL || transport.now_ms() - wait_started < wait_ms) {
        return false;
    }
    if (init_step == needed) {
        return true;
    }

    if (is_grove_aip31068()) {
        const uint8_t command[] = {0x80, grove_init_cmds[init_step]};
        if (!transport.write(transport.context, lcd_address, command, sizeof command)) {
            reset_display(500);
            return false;
        }
        wait_for(grove_init_waits[init_step]);
    } else {
        bool ok = (init_step < 4U)
            ? pcf_write_nibble(pcf_init_cmds[init_step], 0U)
            : pcf_write_byte(pcf_init_cmds[init_step], 0U);
        if (!ok) {
            reset_display(500);
            return false;
        }
        wait_for(pcf_init_waits[init_step]);
    }

    ++init_step;
    return false;
}

bool lcd_write_line(uint8_t row, const char *text)
{
    char padded[LCD_COLUMNS];
    uint8_t cursor_cmd;

    if (row >= LCD_ROWS || text == NULL || init_step != total_init_steps() ||
        transport.now_ms() - wait_started < wait_ms) {
        return false;
    }

    memset(padded, ' ', LCD_COLUMNS);
    for (uint8_t col = 0; col < LCD_COLUMNS && text[col] != '\0'; ++col) {
        padded[col] = text[col];
    }

    if (row_valid[row] && memcmp(displayed[row], padded, LCD_COLUMNS) == 0) {
        return true;
    }

    cursor_cmd = (row == 0U) ? 0x80U : 0xC0U;

    if (is_grove_aip31068()) {
        uint8_t frame[3 + LCD_COLUMNS] = {0x80U, cursor_cmd, 0x40U};
        memcpy(frame + 3, padded, LCD_COLUMNS);
        if (!transport.write(transport.context, lcd_address, frame, sizeof frame)) {
            reset_display(500);
            return false;
        }
    } else {
        if (!pcf_write_byte(cursor_cmd, 0U)) {
            reset_display(500);
            return false;
        }
        for (uint8_t col = 0; col < LCD_COLUMNS; ++col) {
            if (!pcf_write_byte((uint8_t)padded[col], PCF_RS)) {
                reset_display(500);
                return false;
            }
        }
    }

    memcpy(displayed[row], padded, LCD_COLUMNS);
    row_valid[row] = true;
    return true;
}
