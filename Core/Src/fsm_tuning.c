#include "fsm_tuning.h"
#include "app_internal.h"

void fsm_tuning_enter(void)
{
    app_timer_cancel();
    app_set_phase(PHASE_ALL_RED);
    app_state.draft = app_timing;
    app_state.selected = PARAM_RED;
    app_state.tuning_status = TUNING_EDITING;
}

void fsm_tuning_run(app_event_t b2_event, app_event_t b3_event, app_event_t b4_event)
{
    /* B2 long press (1 s): validate R = G + Y & limits, then commit or mark invalid */
    if (b2_event == APP_EVENT_LONG) {
        if (timing_validate(&app_state.draft)) {
            app_timing = app_state.draft;
            app_state.tuning_status = TUNING_SAVED;
        } else {
            app_state.tuning_status = TUNING_INVALID;
        }
        return;
    }

    /* B2 short press: select next parameter R -> G -> Y -> R */
    if (b2_event == APP_EVENT_PRESS) {
        switch (app_state.selected) {
        case PARAM_RED:   app_state.selected = PARAM_GREEN; break;
        case PARAM_GREEN: app_state.selected = PARAM_AMBER; break;
        case PARAM_AMBER:
        default:          app_state.selected = PARAM_RED;   break;
        }
        app_state.tuning_status = TUNING_EDITING;
        return;
    }

    /* B3 press: increase selected draft duration by 1 s within valid bounds */
    if (b3_event == APP_EVENT_PRESS) {
        switch (app_state.selected) {
        case PARAM_RED:
            if (app_state.draft.red_s < 70U) ++app_state.draft.red_s;
            break;
        case PARAM_GREEN:
            if (app_state.draft.green_s < 60U) ++app_state.draft.green_s;
            break;
        case PARAM_AMBER:
            if (app_state.draft.amber_s < 10U) ++app_state.draft.amber_s;
            break;
        default:
            break;
        }
        app_state.tuning_status = TUNING_EDITING;
        return;
    }

    /* B4 press: decrease selected draft duration by 1 s within valid bounds */
    if (b4_event == APP_EVENT_PRESS) {
        switch (app_state.selected) {
        case PARAM_RED:
            if (app_state.draft.red_s > 2U) --app_state.draft.red_s;
            break;
        case PARAM_GREEN:
            if (app_state.draft.green_s > 1U) --app_state.draft.green_s;
            break;
        case PARAM_AMBER:
            if (app_state.draft.amber_s > 1U) --app_state.draft.amber_s;
            break;
        default:
            break;
        }
        app_state.tuning_status = TUNING_EDITING;
        return;
    }
}

void fsm_tuning_exit(void)
{
    app_timer_cancel();
}
