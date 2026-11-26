#include "fsm_automatic.h"
#include "app_internal.h"
static void start_phase(app_phase_t phase)
{
    uint32_t seconds;
    app_set_phase(phase);
    if (app_state.fault) return;
    seconds = (phase == PHASE_P0 || phase == PHASE_P2)
        ? app_timing.green_s : app_timing.amber_s;
    (void)app_timer_start(seconds * 1000U);
}
void fsm_automatic_enter(void)
{
    app_timer_cancel();
    app_set_phase(PHASE_ALL_RED);
    if (!app_state.fault) (void)app_timer_start(1000U);
}
void fsm_automatic_run(app_event_t event)
{
    (void)event;
    if (!app_timer_expired()) return;
    switch (app_state.phase) {
    case PHASE_ALL_RED: case PHASE_P3: start_phase(PHASE_P0); break;
    case PHASE_P0: start_phase(PHASE_P1); break;
    case PHASE_P1: start_phase(PHASE_P2); break;
    case PHASE_P2: start_phase(PHASE_P3); break;
    default: app_fail_safe(); break;
    }
}
void fsm_automatic_exit(void) { app_timer_cancel(); }
