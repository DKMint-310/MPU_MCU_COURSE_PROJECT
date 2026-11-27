#ifndef FSM_TUNING_H
#define FSM_TUNING_H

#include "global.h"

void fsm_tuning_enter(void);
void fsm_tuning_run(app_event_t b2_event, app_event_t b3_event, app_event_t b4_event);
void fsm_tuning_exit(void);

#endif
