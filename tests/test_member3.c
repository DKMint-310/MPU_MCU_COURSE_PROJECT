#include "app_control.h"
#include "fsm_automatic.h"
#include "fsm_manual.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

static uint32_t remaining;
static bool expired, fail_timer, fail_output;
static app_light_t road1, road2;

static app_event_t take(uint8_t id)
{
    assert(id < 4);
    return APP_EVENT_NONE;
}

static bool start(uint8_t id, uint32_t ms)
{
    assert(id == 0);
    remaining = ms;
    expired = false;
    return !fail_timer;
}

static void cancel(uint8_t id)
{
    assert(id == 0);
    remaining = 0;
    expired = false;
}

static bool timeout(uint8_t id)
{
    bool result = expired;
    assert(id == 0);
    expired = false;
    return result;
}

static uint32_t left(uint8_t id)
{
    assert(id == 0);
    return remaining;
}

static void all_red(void)
{
    road1 = APP_RED;
    road2 = APP_RED;
}

static bool apply(app_light_t a, app_light_t b)
{
    assert(a == APP_RED || b == APP_RED);
    road1 = a;
    road2 = b;
    return !fail_output;
}

static app_services_t services = {
    take, start, cancel, timeout, apply, all_red, left
};

static void reset(void)
{
    fail_timer = fail_output = false;
    services.timer_remaining_ms = left;
    app_timing = (timing_config_t){7, 5, 2};
    assert(app_control_init(&services));
}

static void elapse(uint32_t ms)
{
    while (ms >= 10) {
        if (remaining) {
            remaining = remaining > 10 ? remaining - 10 : 0;
            if (!remaining) expired = true;
        }
        if (!app_state.fault) fsm_automatic_run(APP_EVENT_NONE);
        ms -= 10;
    }
}

static void check_phase(unsigned phase)
{
    static const app_light_t first[] = {APP_GREEN, APP_AMBER, APP_RED, APP_RED};
    static const app_light_t second[] = {APP_RED, APP_RED, APP_GREEN, APP_AMBER};
    assert(app_get_snapshot().phase == (app_phase_t)phase);
    assert(road1 == first[phase] && road2 == second[phase]);
}

int main(void)
{
    unsigned cycle, phase;
    timing_config_t cfg = {70, 60, 10};
    assert(timing_validate(&cfg));
    cfg.green_s = 0;
    assert(!timing_validate(&cfg));
    cfg = (timing_config_t){8, 5, 2};
    assert(!timing_validate(&cfg));
    assert(!timing_validate(NULL));
    reset();
    assert(app_state.phase == PHASE_ALL_RED);
    assert(road1 == APP_RED && road2 == APP_RED);
    elapse(990);
    assert(app_state.phase == PHASE_ALL_RED);
    elapse(10);
    for (cycle = 0; cycle < 3; ++cycle) {
        for (phase = 0; phase < 4; ++phase) {
            uint32_t duration = phase % 2 ? 2000 : 5000;
            check_phase(phase);
            elapse(duration - 10);
            check_phase(phase);
            assert(app_get_snapshot().phase_remaining_ms == 10);
            elapse(10);
        }
    }
    puts("PASS: AUTO guard, three cycles, exact pairs and simulated deadlines");
    fsm_automatic_exit();
    app_state.mode = APP_MANUAL;
    fsm_manual_enter();
    assert(remaining == 0 && !expired);
    for (phase = 0; phase < 12; ++phase) {
        fsm_manual_run(APP_EVENT_PRESS);
        check_phase(phase % 4);
        fsm_manual_run(APP_EVENT_NONE);
        check_phase(phase % 4);
        fsm_manual_run(APP_EVENT_LONG);
        check_phase(phase % 4);
    }
    fsm_manual_exit();
    puts("PASS: MANUAL one phase per PRESS; NONE/LONG ignored");
    reset();
    app_timing = (timing_config_t){11, 8, 3};
    elapse(1000);
    assert(remaining == 8000);
    elapse(8000);
    assert(remaining == 3000);
    reset(); fail_timer = true; elapse(1000);
    assert(app_state.fault && remaining == 0);
    assert(road1 == APP_RED && road2 == APP_RED);
    reset(); fail_output = true; elapse(1000);
    assert(app_state.fault && road1 == APP_RED && road2 == APP_RED);
    reset(); app_timing.red_s = 99;
    assert(!app_control_init(&services));
    assert(app_state.fault && road1 == APP_RED && road2 == APP_RED);
    reset(); services.timer_remaining_ms = NULL;
    assert(app_control_init(&services));
    assert(!app_get_snapshot().countdown_available);
    assert(!app_control_init(NULL));
    assert(road1 == APP_RED && road2 == APP_RED);
    puts("PASS: timing validation, custom durations, callback failures and optional countdown");
    puts("Desktop modules only: no mode dispatcher, scheduler or physical GPIO test yet.");
    return 0;
}
