#include "software_timer.h"

typedef struct {
    uint32_t remaining_ticks;
    bool active;
    bool expired;
} soft_timer_t;

static soft_timer_t timers[TIMER_COUNT];

void timer_init(void)
{
    uint8_t i;
    for (i = 0U; i < TIMER_COUNT; ++i) {
        timers[i].remaining_ticks = 0U;
        timers[i].active = false;
        timers[i].expired = false;
    }
}

bool timer_start(uint8_t id, uint32_t ms)
{
    uint32_t ticks;

    if (id >= TIMER_COUNT || ms == 0U) {
        return false;
    }

    /* Round milliseconds > 0 up to 10 ms ticks without overflow */
    ticks = (ms / TIMER_TICK_MS) + (((ms % TIMER_TICK_MS) != 0U) ? 1U : 0U);

    timers[id].remaining_ticks = ticks;
    timers[id].active = true;
    timers[id].expired = false;
    return true;
}

void timer_cancel(uint8_t id)
{
    if (id >= TIMER_COUNT) {
        return;
    }

    timers[id].remaining_ticks = 0U;
    timers[id].active = false;
    timers[id].expired = false;
}

bool timer_take_expired(uint8_t id)
{
    if (id >= TIMER_COUNT || !timers[id].expired) {
        return false;
    }

    timers[id].expired = false;
    return true;
}

uint32_t timer_remaining_ms(uint8_t id)
{
    uint32_t ticks;

    if (id >= TIMER_COUNT || !timers[id].active) {
        return 0U;
    }

    ticks = timers[id].remaining_ticks;
    if (ticks > (UINT32_MAX / TIMER_TICK_MS)) {
        return UINT32_MAX;
    }
    return ticks * TIMER_TICK_MS;
}

void timer_tick(void)
{
    uint8_t i;
    for (i = 0U; i < TIMER_COUNT; ++i) {
        if (timers[i].active && timers[i].remaining_ticks > 0U) {
            --timers[i].remaining_ticks;
            if (timers[i].remaining_ticks == 0U) {
                timers[i].active = false;
                timers[i].expired = true;
            }
        }
    }
}
