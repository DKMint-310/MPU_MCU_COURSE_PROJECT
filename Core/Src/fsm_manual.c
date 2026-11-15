#include "fsm_manual.h"
#include "app_internal.h"
void fsm_manual_enter(void)
{
    app_timer_cancel();
    app_set_phase(PHASE_ALL_RED);
}
void fsm_manual_run(app_event_t event)
{
    if (event != APP_EVENT_PRESS) return;
    switch (app_state.phase) {
    case PHASE_ALL_RED: case PHASE_P3: app_set_phase(PHASE_P0); break;
    case PHASE_P0: app_set_phase(PHASE_P1); break;
    case PHASE_P1: app_set_phase(PHASE_P2); break;
    case PHASE_P2: app_set_phase(PHASE_P3); break;
    default: app_fail_safe(); break;
    }
}
void fsm_manual_exit(void) { app_timer_cancel(); }
