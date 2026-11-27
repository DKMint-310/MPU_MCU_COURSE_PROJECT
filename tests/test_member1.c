#include "scheduler.h"
#include "software_timer.h"
#include "button.h"
#include "traffic_light.h"
#include "dht20.h"
#include "lcd.h"
#include "display.h"
#include "global.h"
#include "app_control.h"
#include "fsm_automatic.h"
#include "fsm_manual.h"
#include "fsm_tuning.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/* 1. Scheduler & Software Timer Unit Tests (S01-S06, D01)                    */
/* ========================================================================== */

static uint32_t g_tick;
static uint32_t pa5_cnt, pc9_cnt, oneshot_cnt, reclaim_cnt;
static uint8_t order_log[8], order_len;
static uint32_t self_del_id, self_del_runs;

static void cb_pa5(void) { assert(g_tick % 50U == 0U); ++pa5_cnt; }
static void cb_pc9(void) { assert(g_tick % 100U == 0U); ++pc9_cnt; }
static void cb_oneshot(void) { assert(g_tick == 300U); ++oneshot_cnt; }
static void cb_noop(void) {}
static void cb_reclaim(void) { ++reclaim_cnt; }

static void cb_oneshot_full(void)
{
    assert(SCH_Get_Task_Count() == SCH_MAX_TASKS - 1U);
    assert(SCH_Add_Task(cb_reclaim, 2U, 0U) != SCH_NO_TASK_ID);
}

static void rec0(void) { order_log[order_len++] = 0U; }
static void rec1(void) { order_log[order_len++] = 1U; }
static void rec2(void) { order_log[order_len++] = 2U; }
static void rec3(void) { order_log[order_len++] = 3U; }

static void cb_self_del(void)
{
    if (++self_del_runs == 3U) {
        assert(SCH_Delete_Task(self_del_id) == 1U);
        assert(SCH_Delete_Task(self_del_id) == 0U);
    }
}

static void test_scheduler_and_timers(void)
{
    SCH_Init();
    pa5_cnt = pc9_cnt = oneshot_cnt = g_tick = 0U;
    assert(SCH_Add_Task(cb_pa5, 0U, 50U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(cb_pc9, 0U, 100U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(cb_oneshot, 300U, 0U) != SCH_NO_TASK_ID);

    SCH_Dispatch_Tasks();
    assert(pa5_cnt == 1U && pc9_cnt == 1U && oneshot_cnt == 0U);

    for (g_tick = 1U; g_tick <= 1000U; ++g_tick) {
        SCH_Update();
        SCH_Dispatch_Tasks();
    }
    assert(pa5_cnt == 21U && pc9_cnt == 11U && oneshot_cnt == 1U);

    /* One-shot releases slot before callback */
    SCH_Init();
    reclaim_cnt = 0U;
    assert(SCH_Add_Task(cb_oneshot_full, 5U, 0U) != SCH_NO_TASK_ID);
    for (uint32_t i = 1U; i < SCH_MAX_TASKS; ++i) {
        assert(SCH_Add_Task(cb_noop, 100U + i, 0U) != SCH_NO_TASK_ID);
    }
    assert(SCH_Add_Task(cb_noop, 10U, 0U) == SCH_NO_TASK_ID);
    for (g_tick = 1U; g_tick <= 7U; ++g_tick) {
        SCH_Update();
        SCH_Dispatch_Tasks();
    }
    assert(reclaim_cnt == 1U);

    /* Head/middle/tail delete + equal-deadline FIFO */
    SCH_Init();
    order_len = 0U;
    uint32_t id_h = SCH_Add_Task(rec0, 5U, 0U);
    uint32_t id_m1 = SCH_Add_Task(rec1, 15U, 0U);
    uint32_t id_m2 = SCH_Add_Task(rec2, 25U, 0U);
    uint32_t id_t = SCH_Add_Task(rec3, 35U, 0U);
    (void)id_m1;
    assert(SCH_Delete_Task(id_h) == 1U);
    assert(SCH_Delete_Task(id_m2) == 1U);
    assert(SCH_Delete_Task(id_t) == 1U);
    for (g_tick = 1U; g_tick <= 40U; ++g_tick) {
        SCH_Update();
        SCH_Dispatch_Tasks();
    }
    assert(order_len == 1U && order_log[0] == 1U);

    SCH_Init();
    order_len = 0U;
    assert(SCH_Add_Task(rec0, 5U, 0U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(rec1, 5U, 0U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(rec2, 5U, 0U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(rec3, 5U, 0U) != SCH_NO_TASK_ID);
    for (g_tick = 1U; g_tick <= 5U; ++g_tick) {
        SCH_Update();
        SCH_Dispatch_Tasks();
    }
    assert(order_len == 4U && order_log[0] == 0U && order_log[3] == 3U);

    /* Periodic self-delete suppresses reinsertion */
    SCH_Init();
    self_del_runs = 0U;
    self_del_id = SCH_Add_Task(cb_self_del, 3U, 3U);
    for (g_tick = 1U; g_tick <= 30U; ++g_tick) {
        SCH_Update();
        SCH_Dispatch_Tasks();
    }
    assert(self_del_runs == 3U && SCH_Get_Task_Count() == 0U);

    /* Software timers: 1/10/11 ms ceiling rounding + one-time flag */
    timer_init();
    assert(!timer_start(0U, 0U));
    assert(timer_start(0U, 1U) && timer_start(1U, 10U) && timer_start(2U, 11U));
    timer_tick();
    assert(timer_take_expired(0U) && !timer_take_expired(0U));
    assert(timer_take_expired(1U) && !timer_take_expired(1U));
    assert(!timer_take_expired(2U) && timer_remaining_ms(2U) == 10U);
    timer_tick();
    assert(timer_take_expired(2U) && !timer_take_expired(2U));

    puts("PASS [1/2]: Scheduler O(1) queue & Software Timers verified");
}

/* ========================================================================== */
/* 2. Full End-to-End System Test (AUTO -> MANUAL -> TUNING -> AUTO + LCD/I2C)*/
/* ========================================================================== */

static uint32_t sim_now_ms;
static bool raw_btn_pin[BUTTON_COUNT]; /* Active-LOW electrical levels */
static uint8_t gpio_road_bits[2];
static char pcf_lcd_screen[LCD_ROWS][LCD_COLUMNS + 1U];
static uint8_t pcf_cursor_row, pcf_cursor_col;
static bool pcf_nibble_phase;
static uint8_t pcf_upper_latch;
static bool pcf_4bit_ready;

static bool sim_read_btn(void *ctx, uint8_t id)
{
    (void)ctx;
    return raw_btn_pin[id];
}

static void sim_write_light_pin(void *ctx, uint8_t road, uint8_t pin, bool high)
{
    uint8_t mask = (pin == 0U) ? 2U : 1U;
    (void)ctx;
    if (high) {
        gpio_road_bits[road] |= mask;
    } else {
        gpio_road_bits[road] &= (uint8_t)~mask;
    }
    /* Physical safety check: NEVER both roads green (code 2U) simultaneously! */
    assert(!(gpio_road_bits[0] == 2U && gpio_road_bits[1] == 2U));
}

static uint8_t dht20_crc8(const uint8_t data[6])
{
    uint8_t crc = 0xFFU;
    for (uint8_t i = 0U; i < 6U; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0U; b < 8U; ++b) {
            crc = (uint8_t)((crc << 1U) ^ ((crc & 0x80U) ? 0x31U : 0U));
        }
    }
    return crc;
}

/* Simulates both DHT20 (0x38) and PCF8574 HD44780 16x2 LCD (0x21 OhStem) on shared I2C */
static bool sim_i2c_write(void *ctx, uint8_t addr, const uint8_t *data, uint16_t size)
{
    (void)ctx;
    if (addr == DHT20_ADDRESS) {
        return true;
    }
    if (addr == LCD_OHSTEM_ADDRESS) {
        for (uint16_t i = 0U; i < size; ++i) {
            uint8_t b = data[i];
            /* Latch on falling edge of EN (0x04) — check when EN is high */
            if ((b & 0x04U) != 0U) {
                bool rs = (b & 0x01U) != 0U;
                if (!pcf_4bit_ready) {
                    if ((b & 0xF0U) == 0x20U) {
                        pcf_4bit_ready = true;
                        pcf_nibble_phase = false;
                    }
                } else if (!pcf_nibble_phase) {
                    pcf_upper_latch = (uint8_t)(b & 0xF0U);
                    pcf_nibble_phase = true;
                } else {
                    uint8_t full = (uint8_t)(pcf_upper_latch | ((b >> 4U) & 0x0FU));
                    pcf_nibble_phase = false;
                    if (!rs) {
                        if (full == 0x01U) {
                            memset(pcf_lcd_screen, ' ', sizeof pcf_lcd_screen);
                            pcf_lcd_screen[0][16] = '\0';
                            pcf_lcd_screen[1][16] = '\0';
                        } else if (full == 0x80U) {
                            pcf_cursor_row = 0U;
                            pcf_cursor_col = 0U;
                        } else if (full == 0xC0U) {
                            pcf_cursor_row = 1U;
                            pcf_cursor_col = 0U;
                        }
                    } else if (pcf_cursor_row < LCD_ROWS && pcf_cursor_col < LCD_COLUMNS) {
                        pcf_lcd_screen[pcf_cursor_row][pcf_cursor_col++] = (char)full;
                        pcf_lcd_screen[pcf_cursor_row][16] = '\0';
                    }
                }
            }
        }
        return true;
    }
    if (addr == LCD_GROVE_ADDRESS) {
        return true;
    }
    return false;
}

static bool sim_i2c_read(void *ctx, uint8_t addr, uint8_t *data, uint16_t size)
{
    (void)ctx;
    if (addr != DHT20_ADDRESS) {
        return false;
    }
    if (size == 1U) {
        data[0] = 0x18U; /* Calibrated & ready */
        return true;
    }
    if (size == 7U) {
        /* 50.0% RH (0x80000), 25.0 C ((25+50)/200 * 2^20 = 393216 = 0x60000) */
        data[0] = 0x18U;
        data[1] = 0x80U;
        data[2] = 0x00U;
        data[3] = 0x06U;
        data[4] = 0x00U;
        data[5] = 0x00U;
        data[6] = dht20_crc8(data);
        return true;
    }
    return false;
}

static uint32_t sim_get_ms(void) { return sim_now_ms; }

static app_event_t bridge_btn(uint8_t id)
{
    switch (button_take_event(id)) {
    case BUTTON_PRESS: return APP_EVENT_PRESS;
    case BUTTON_LONG:  return APP_EVENT_LONG;
    default:           return APP_EVENT_NONE;
    }
}

static light_t bridge_light_enum(app_light_t l)
{
    switch (l) {
    case APP_GREEN: return LIGHT_GREEN;
    case APP_AMBER: return LIGHT_AMBER;
    default:        return LIGHT_RED;
    }
}

static bool bridge_lights(app_light_t r1, app_light_t r2)
{
    return traffic_apply_pair(bridge_light_enum(r1), bridge_light_enum(r2));
}

static display_view_t bridge_view(void)
{
    app_snapshot_t s = app_get_snapshot();
    display_view_t v = {0};
    v.mode = (s.mode == APP_MANUAL) ? DISPLAY_MANUAL
           : (s.mode == APP_TUNING) ? DISPLAY_TUNING : DISPLAY_AUTO;
    switch (s.phase) {
    case PHASE_P0: v.phase = DISPLAY_ROAD1_GREEN; break;
    case PHASE_P1: v.phase = DISPLAY_ROAD1_AMBER; break;
    case PHASE_P2: v.phase = DISPLAY_ROAD2_GREEN; break;
    case PHASE_P3: v.phase = DISPLAY_ROAD2_AMBER; break;
    default:       v.phase = DISPLAY_ALL_RED;     break;
    }
    v.remaining_s = (uint16_t)((s.phase_remaining_ms + 999U) / 1000U);
    v.tuning_field = (s.selected == PARAM_GREEN) ? 'G'
                   : (s.selected == PARAM_AMBER) ? 'Y' : 'R';
    v.tuning_value_s = (s.selected == PARAM_GREEN) ? s.draft.green_s
                     : (s.selected == PARAM_AMBER) ? s.draft.amber_s : s.draft.red_s;
    v.tuning_status = (s.tuning_status == TUNING_SAVED)   ? DISPLAY_SAVED
                    : (s.tuning_status == TUNING_INVALID) ? DISPLAY_INVALID : DISPLAY_EDITING;
    return v;
}

static void step_system_ms(uint32_t ms)
{
    while (ms >= 10U) {
        sim_now_ms += 10U;
        SCH_Update();
        SCH_Dispatch_Tasks();
        ms -= 10U;
    }
}

/* Simulates a noisy physical active-LOW button press + release */
static void press_short(uint8_t btn_id)
{
    raw_btn_pin[btn_id] = false; step_system_ms(10U); /* bounce low */
    raw_btn_pin[btn_id] = true;  step_system_ms(10U); /* bounce high */
    raw_btn_pin[btn_id] = false; step_system_ms(50U); /* stable press (50 ms) */
    raw_btn_pin[btn_id] = true;  step_system_ms(30U); /* 3-sample release -> event fires */
}

/* Simulates holding a button >= 1.0 s then releasing */
static void press_long_1s(uint8_t btn_id)
{
    raw_btn_pin[btn_id] = false;
    step_system_ms(1050U); /* 30 ms debounce + 1000 ms hold -> emits LONG */
    raw_btn_pin[btn_id] = true;
    step_system_ms(40U);   /* release must NOT emit an extra short PRESS */
}

static void test_full_system_end_to_end(void)
{
    static const bool active_high[4] = {false, false, false, false};
    static const traffic_codes_t codes[2] = {{3U, 1U, 2U}, {3U, 1U, 2U}};
    static const app_services_t services = {
        bridge_btn, timer_start, timer_cancel, timer_take_expired,
        bridge_lights, traffic_all_red, timer_remaining_ms
    };
    peripheral_i2c_t bus = {NULL, sim_i2c_write, sim_i2c_read, sim_get_ms};

    sim_now_ms = 0U;
    for (int i = 0; i < 4; ++i) raw_btn_pin[i] = true; /* idle HIGH (GPIO_PULLUP) */
    gpio_road_bits[0] = 3U;
    gpio_road_bits[1] = 3U;
    pcf_4bit_ready = false;
    pcf_nibble_phase = false;
    memset(pcf_lcd_screen, 0, sizeof pcf_lcd_screen);
    app_timing = (timing_config_t){7U, 5U, 2U};

    SCH_Init();
    timer_init();
    assert(button_init(sim_read_btn, NULL, active_high));
    assert(traffic_light_init(sim_write_light_pin, NULL, codes));
    assert(dht20_init(&bus));
    assert(lcd_configure(&bus, LCD_OHSTEM_ADDRESS));
    assert(display_init(bridge_view));
    assert(app_control_init(&services));

    assert(SCH_Add_Task(timer_tick, 1U, 1U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(button_sample, 1U, 1U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(app_control_run, 1U, 1U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(dht20_task, 1U, 1U) != SCH_NO_TASK_ID);
    assert(SCH_Add_Task(display_task, 1U, 20U) != SCH_NO_TASK_ID);

    /* 1. Boot ALL_RED 1s -> P0 (G1/R2 5s) -> P1 (Y1/R2 2s) */
    step_system_ms(1000U);
    assert(app_get_snapshot().mode == APP_AUTO && app_get_snapshot().phase == PHASE_P0);
    assert(gpio_road_bits[0] == 2U && gpio_road_bits[1] == 3U); /* Road1 GREEN, Road2 RED */

    step_system_ms(5000U);
    assert(app_get_snapshot().phase == PHASE_P1);
    assert(gpio_road_bits[0] == 1U && gpio_road_bits[1] == 3U); /* Road1 AMBER, Road2 RED */

    /* Verify PCF8574 LCD initialized & rendered both AUTO line and DHT20 environment! */
    assert(strncmp(pcf_lcd_screen[0], "AUTO ", 5) == 0);
    assert(strncmp(pcf_lcd_screen[1], "T25.0C H50%", 11) == 0);

    /* 2. Press B1 while light is AMBER -> must safely enter MANUAL at ALL_RED */
    press_short(0U);
    assert(app_get_snapshot().mode == APP_MANUAL);
    assert(app_get_snapshot().phase == PHASE_ALL_RED);
    assert(gpio_road_bits[0] == 3U && gpio_road_bits[1] == 3U);

    /* Press B2 three times in MANUAL: P0 -> P1 -> P2 (Road1 RED, Road2 GREEN) */
    press_short(1U);
    assert(app_get_snapshot().phase == PHASE_P0);
    press_short(1U);
    assert(app_get_snapshot().phase == PHASE_P1);
    press_short(1U);
    assert(app_get_snapshot().phase == PHASE_P2);
    assert(gpio_road_bits[0] == 3U && gpio_road_bits[1] == 2U);

    /* 3. Press B1 -> enter TUNING (ALL_RED, draft = {7, 5, 2}, selected = RED) */
    press_short(0U);
    assert(app_get_snapshot().mode == APP_TUNING);
    assert(app_get_snapshot().phase == PHASE_ALL_RED);
    assert(gpio_road_bits[0] == 3U && gpio_road_bits[1] == 3U);

    /* Increase R from 7 -> 8 via B3, then hold B2 1s -> INVALID (8 != 5 + 2)! */
    press_short(2U);
    assert(app_get_snapshot().draft.red_s == 8U);
    press_long_1s(1U);
    assert(app_get_snapshot().tuning_status == TUNING_INVALID);
    assert(app_timing.red_s == 7U); /* Committed config untouched! */

    /* Short-press B2 to select GREEN, increase G from 5 -> 6 via B3, hold B2 1s -> SAVED! */
    press_short(1U);
    assert(app_get_snapshot().selected == PARAM_GREEN);
    press_short(2U);
    assert(app_get_snapshot().draft.green_s == 6U);
    press_long_1s(1U);
    assert(app_get_snapshot().tuning_status == TUNING_SAVED);
    assert(app_timing.red_s == 8U && app_timing.green_s == 6U && app_timing.amber_s == 2U);

    /* Make an unsaved edit (G: 6 -> 5 via B4), then press B1 to CANCEL & return to AUTO */
    press_short(3U);
    assert(app_get_snapshot().draft.green_s == 5U);
    press_short(0U);
    assert(app_get_snapshot().mode == APP_AUTO);
    assert(app_get_snapshot().phase == PHASE_ALL_RED);
    assert(app_timing.green_s == 6U); /* Unsaved draft discarded, saved 6s preserved! */

    /* 4. Verify AUTO now runs with the newly saved 6s GREEN duration! */
    step_system_ms(1000U); /* Exit 1s ALL_RED guard -> P0 */
    assert(app_get_snapshot().phase == PHASE_P0);
    step_system_ms(5990U);
    assert(app_get_snapshot().phase == PHASE_P0); /* Still GREEN at 5.99s! */
    step_system_ms(10U);
    assert(app_get_snapshot().phase == PHASE_P1); /* Transitions to AMBER at exact 6.00s! */

    /* 5. Verify Grove 0x3E LCD fallback protocol also initializes & writes cleanly */
    assert(lcd_configure(&bus, LCD_GROVE_ADDRESS));
    for (int k = 0; k < 20; ++k) {
        sim_now_ms += 60U;
        if (lcd_init()) break;
    }
    assert(lcd_init());
    assert(lcd_write_line(0U, "GROVE OK"));

    puts("PASS [2/2]: Full System (AUTO/MANUAL/TUNING + Debounce + DHT20 + PCF8574/Grove LCD) verified");
}

int main(void)
{
    test_scheduler_and_timers();
    test_full_system_end_to_end();
    puts("ALL SYSTEM TESTS PASSED.");
    return 0;
}
