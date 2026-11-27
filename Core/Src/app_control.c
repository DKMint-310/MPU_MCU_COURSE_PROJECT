#include "app_internal.h"
#include "fsm_automatic.h"
#include "fsm_manual.h"
#include "fsm_tuning.h"
#include <stddef.h>

static app_services_t io;
static bool initialized;
enum { PHASE_TIMER = 0 };

void app_timer_cancel(void) { io.timer_cancel(PHASE_TIMER); }
void app_fail_safe(void)
{
    app_state.fault = true;
    app_state.phase = PHASE_ALL_RED;
    app_state.phase_remaining_ms = 0;
    app_timer_cancel();
    io.traffic_all_red();
}
bool app_timer_start(uint32_t ms)
{
    if (io.timer_start(PHASE_TIMER, ms)) return true;
    app_fail_safe();
    return false;
}
bool app_timer_expired(void) { return io.timer_take_expired(PHASE_TIMER); }
void app_set_phase(app_phase_t phase)
{
    app_light_t a = APP_RED, b = APP_RED;
    switch (phase) {
    case PHASE_P0: a = APP_GREEN; break;
    case PHASE_P1: a = APP_AMBER; break;
    case PHASE_P2: b = APP_GREEN; break;
    case PHASE_P3: b = APP_AMBER; break;
    case PHASE_ALL_RED: break;
    default: app_fail_safe(); return;
    }
    if (app_state.fault) return;

    if (!io.traffic_apply_pair(a, b)) { app_fail_safe(); return; }
    app_state.phase = phase;
}
bool app_control_init(const app_services_t *services)
{
    if (initialized) { app_timer_cancel(); io.traffic_all_red(); }
    initialized = false;
    if (services == NULL || services->button_take_event == NULL
        || services->timer_start == NULL || services->timer_cancel == NULL
        || services->timer_take_expired == NULL
        || services->traffic_apply_pair == NULL || services->traffic_all_red == NULL)
        return false;
    io = *services;
    initialized = true;
    app_state = (app_snapshot_t){0};
    app_state.countdown_available = io.timer_remaining_ms != NULL;
    io.traffic_all_red();
    if (!timing_validate(&app_timing)) { app_fail_safe(); return false; }
    app_state.mode = APP_AUTO;
    fsm_automatic_enter();
    return !app_state.fault;
}

void app_control_run(void)
{
    app_event_t b1, b2, b3, b4;

    if (!initialized) {
        return;
    }

    /* Consume button events once per 10 ms control tick */
    b1 = io.button_take_event(0U);
    b2 = io.button_take_event(1U);
    b3 = io.button_take_event(2U);
    b4 = io.button_take_event(3U);

    /* B1 has highest priority: switch AUTO -> MANUAL -> TUNING -> AUTO */
    if (b1 == APP_EVENT_PRESS) {
        app_state.fault = false;
        switch (app_state.mode) {
        case APP_AUTO:
            fsm_automatic_exit();
            app_state.mode = APP_MANUAL;
            fsm_manual_enter();
            break;
        case APP_MANUAL:
            fsm_manual_exit();
            app_state.mode = APP_TUNING;
            fsm_tuning_enter();
            break;
        case APP_TUNING:
        default:
            fsm_tuning_exit();
            app_state.mode = APP_AUTO;
            fsm_automatic_enter();
            break;
        }
        return;
    }

    if (app_state.fault) {
        return;
    }

    switch (app_state.mode) {
    case APP_AUTO:
        fsm_automatic_run(APP_EVENT_NONE);
        break;
    case APP_MANUAL:
        fsm_manual_run(b2);
        break;
    case APP_TUNING:
        fsm_tuning_run(b2, b3, b4);
        break;
    default:
        app_fail_safe();
        break;
    }
}

app_snapshot_t app_get_snapshot(void)
{
    app_snapshot_t result = app_state;
    result.committed = app_timing;
    result.phase_remaining_ms = 0;
    if (initialized && !result.fault && result.mode == APP_AUTO
        && io.timer_remaining_ms != NULL)
        result.phase_remaining_ms = io.timer_remaining_ms(PHASE_TIMER);
    return result;
}
