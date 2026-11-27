#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DISPLAY_AUTO,
    DISPLAY_MANUAL,
    DISPLAY_TUNING
} display_mode_t;

typedef enum {
    DISPLAY_ALL_RED,
    DISPLAY_ROAD1_GREEN,
    DISPLAY_ROAD1_AMBER,
    DISPLAY_ROAD2_GREEN,
    DISPLAY_ROAD2_AMBER
} display_phase_t;

typedef enum {
    DISPLAY_EDITING,
    DISPLAY_SAVED,
    DISPLAY_INVALID
} display_status_t;

/* Member 3 maps the application snapshot to this view. */
typedef struct {
    display_mode_t mode;
    display_phase_t phase;
    uint16_t remaining_s;
    char tuning_field; /* R, G or Y. */
    uint16_t tuning_value_s;
    display_status_t tuning_status;
} display_view_t;

typedef display_view_t (*display_read_view_fn)(void);

bool display_init(display_read_view_fn read_view);
/* Schedule every 200 ms. Advances LCD initialization or refreshes one row. */
void display_task(void);

#endif
