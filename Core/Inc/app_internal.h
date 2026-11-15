#ifndef APP_INTERNAL_H
#define APP_INTERNAL_H
#include "app_control.h"

void app_set_phase(app_phase_t phase);
void app_timer_cancel(void);
bool app_timer_start(uint32_t ms);
bool app_timer_expired(void);
void app_fail_safe(void);
#endif
