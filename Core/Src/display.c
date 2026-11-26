#include "display.h"

#include "dht20.h"
#include "lcd.h"

#include <stddef.h>
#include <stdio.h>

enum { LINE_BUFFER_SIZE = 32 };

static display_read_view_fn get_view;
static uint8_t next_row;
static uint8_t status_slot;
static display_status_t last_status;

bool display_init(display_read_view_fn read_view)
{
    if (read_view == NULL) {
        return false;
    }
    get_view = read_view;
    next_row = 0;
    status_slot = 0;
    last_status = DISPLAY_EDITING;
    return true;
}

static const char *phase_text(display_phase_t phase)
{
    switch (phase) {
    case DISPLAY_ROAD1_GREEN: return "R1 G";
    case DISPLAY_ROAD1_AMBER: return "R1 Y";
    case DISPLAY_ROAD2_GREEN: return "R2 G";
    case DISPLAY_ROAD2_AMBER: return "R2 Y";
    default: return "ALL RED";
    }
}

static void format_mode(char *line, const display_view_t *view)
{
    if (view->mode == DISPLAY_TUNING) {
        (void)snprintf(line, LINE_BUFFER_SIZE, "SET %c %us",
                       view->tuning_field, (unsigned)view->tuning_value_s);
    } else if (view->mode == DISPLAY_MANUAL) {
        (void)snprintf(line, LINE_BUFFER_SIZE, "MAN %s", phase_text(view->phase));
    } else {
        unsigned remaining = view->remaining_s > 99U ? 99U : view->remaining_s;
        (void)snprintf(line, LINE_BUFFER_SIZE, "AUTO %s %us",
                       phase_text(view->phase), remaining);
    }
}

static void format_environment(char *line)
{
    env_sample_t env = dht20_get_latest();
    if (!env.valid) {
        (void)snprintf(line, LINE_BUFFER_SIZE, "T-- C  H--%%");
        return;
    }

    float scaled = env.temperature_c * 10.0f;
    int tenths = (int)(scaled + (scaled < 0.0f ? -0.5f : 0.5f));
    unsigned absolute = (unsigned)(tenths < 0 ? -tenths : tenths);
    unsigned humidity = (unsigned)(env.humidity_pct + 0.5f);
    (void)snprintf(line, LINE_BUFFER_SIZE, "T%s%u.%uC H%u%%%s",
                   tenths < 0 ? "-" : "", absolute / 10U, absolute % 10U,
                   humidity, env.stale ? "*" : "");
}

void display_task(void)
{
    if (get_view == NULL || !lcd_init()) {
        return;
    }

    display_view_t view = get_view();
    char line[LINE_BUFFER_SIZE];
    if (view.tuning_status != last_status) {
        status_slot = 0;
        last_status = view.tuning_status;
    }
    if (next_row == 0) {
        format_mode(line, &view);
    } else if (view.mode == DISPLAY_TUNING && view.tuning_status != DISPLAY_EDITING &&
               status_slot < 5U) {
        (void)snprintf(line, sizeof line, "%s",
                       view.tuning_status == DISPLAY_SAVED ? "SAVED" : "INVALID TIMING");
    } else {
        format_environment(line);
    }

    if (lcd_write_line(next_row, line)) {
        if (next_row == 1) {
            status_slot = (uint8_t)((status_slot + 1U) % 10U);
        }
        next_row ^= 1U;
    }
}
