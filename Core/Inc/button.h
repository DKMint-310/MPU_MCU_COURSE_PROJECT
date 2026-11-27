#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>

#define BUTTON_COUNT 4U

typedef enum {
    BUTTON_NONE,
    BUTTON_PRESS,
    BUTTON_LONG
} button_event_t;

typedef bool (*button_read_fn)(void *context, uint8_t id);

/* read_level returns the raw electrical level. IDs 0-3 are B1-B4. */
bool button_init(button_read_fn read_level, void *context,
                 const bool active_high[BUTTON_COUNT]);
/* Schedule once every 10 ms. */
void button_sample(void);
/* A short press is reported on release. A hold generates one LONG at 1 s. */
button_event_t button_take_event(uint8_t id);

#endif
