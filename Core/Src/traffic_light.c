#include "traffic_light.h"

#include <stddef.h>

static traffic_write_pin_fn write_pin;
static void *write_context;
static traffic_codes_t encoding[2];
static uint8_t current_code[2];

static bool valid_codes(traffic_codes_t codes)
{
    return codes.red < 4 && codes.amber < 4 && codes.green < 4 &&
           codes.red != codes.amber && codes.red != codes.green &&
           codes.amber != codes.green;
}

bool traffic_light_init(traffic_write_pin_fn output, void *context,
                        const traffic_codes_t codes[2])
{
    if (output == NULL || codes == NULL ||
        !valid_codes(codes[0]) || !valid_codes(codes[1])) {
        return false;
    }

    encoding[0] = codes[0];
    encoding[1] = codes[1];
    current_code[0] = codes[0].red;
    current_code[1] = codes[1].red;
    write_pin = output;
    write_context = context;
    return true;
}

static uint8_t encode(uint8_t road, light_t light)
{
    if (light == LIGHT_GREEN) {
        return encoding[road].green;
    }
    return light == LIGHT_AMBER ? encoding[road].amber : encoding[road].red;
}

static void set_pin(uint8_t road, uint8_t mask, uint8_t code)
{
    write_pin(write_context, road, mask == 2U ? 0U : 1U, (code & mask) != 0);
    current_code[road] = (current_code[road] & (uint8_t)~mask) | (code & mask);
}

static void set_road(uint8_t road, uint8_t code)
{
    uint8_t changed = current_code[road] ^ code;

    if (changed == 3U) {
        /* Pins can be on different GPIO ports. Avoid a transient green
         * when both bits must change, even for red/amber transitions. */
        uint8_t first = ((current_code[road] ^ 1U) == encoding[road].green)
                        ? 2U : 1U;
        set_pin(road, first, code);
        set_pin(road, first ^ 3U, code);
    } else if (changed != 0U) {
        set_pin(road, changed, code);
    }
}

static bool valid_light(light_t light)
{
    return light == LIGHT_RED || light == LIGHT_AMBER || light == LIGHT_GREEN;
}

void traffic_all_red(void)
{
    if (write_pin != NULL) {
        set_road(0, encoding[0].red);
        set_road(1, encoding[1].red);
    }
}

bool traffic_apply_pair(light_t road1, light_t road2)
{
    if (write_pin == NULL) {
        return false;
    }
    if (!valid_light(road1) || !valid_light(road2) ||
        (road1 != LIGHT_RED && road2 != LIGHT_RED)) {
        traffic_all_red();
        return false;
    }

    /* Give the yielding road red before changing the other road. */
    if (road1 == LIGHT_RED) {
        set_road(0, encoding[0].red);
        set_road(1, encode(1, road2));
    } else {
        set_road(1, encoding[1].red);
        set_road(0, encode(0, road1));
    }
    return true;
}
