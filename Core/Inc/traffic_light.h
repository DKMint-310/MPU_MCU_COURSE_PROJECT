#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LIGHT_RED,
    LIGHT_AMBER,
    LIGHT_GREEN
} light_t;

typedef struct {
    uint8_t red;
    uint8_t amber;
    uint8_t green;
} traffic_codes_t;

typedef void (*traffic_write_pin_fn)(void *context, uint8_t road, uint8_t pin,
                                     bool high);

/* Codes are (A << 1) | B. The two roads must already be initialized to red.
 * The callback receives zero-based road and pin (0 = A, 1 = B). */
bool traffic_light_init(traffic_write_pin_fn write_pin, void *context,
                        const traffic_codes_t codes[2]);
bool traffic_apply_pair(light_t road1, light_t road2);
void traffic_all_red(void);

#endif
