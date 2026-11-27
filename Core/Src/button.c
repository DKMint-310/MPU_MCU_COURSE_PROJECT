#include "button.h"

#include <stddef.h>
#include <string.h>

enum { DEBOUNCE_SAMPLES = 3, LONG_PRESS_TICKS = 100 };

typedef struct {
    bool active_high;
    bool pressed;
    bool long_sent;
    uint8_t changed_samples;
    uint8_t held_ticks;
    button_event_t event;
} button_state_t;

static button_state_t buttons[BUTTON_COUNT];
static button_read_fn read_pin;
static void *read_context;

bool button_init(button_read_fn read_level, void *context,
                 const bool active_high[BUTTON_COUNT])
{
    if (read_level == NULL || active_high == NULL) {
        return false;
    }

    memset(buttons, 0, sizeof buttons);
    for (uint8_t id = 0; id < BUTTON_COUNT; ++id) {
        buttons[id].active_high = active_high[id];
    }
    read_context = context;
    read_pin = read_level;
    return true;
}

void button_sample(void)
{
    if (read_pin == NULL) {
        return;
    }

    for (uint8_t id = 0; id < BUTTON_COUNT; ++id) {
        button_state_t *button = &buttons[id];
        bool raw_pressed = read_pin(read_context, id) == button->active_high;

        if (raw_pressed != button->pressed) {
            if (++button->changed_samples == DEBOUNCE_SAMPLES) {
                button->changed_samples = 0;
                button->pressed = raw_pressed;
                if (raw_pressed) {
                    button->held_ticks = 0;
                    button->long_sent = false;
                } else if (!button->long_sent) {
                    button->event = BUTTON_PRESS;
                }
                continue;
            }
        } else {
            button->changed_samples = 0;
        }

        if (button->pressed && !button->long_sent &&
            ++button->held_ticks >= LONG_PRESS_TICKS) {
            button->long_sent = true;
            button->event = BUTTON_LONG;
        }
    }
}

button_event_t button_take_event(uint8_t id)
{
    if (id >= BUTTON_COUNT) {
        return BUTTON_NONE;
    }
    button_event_t event = buttons[id].event;
    buttons[id].event = BUTTON_NONE;
    return event;
}
