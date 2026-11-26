#ifndef APP_GLOBAL_H
#define APP_GLOBAL_H
#include <stdbool.h>
#include <stdint.h>

typedef enum { APP_AUTO, APP_MANUAL, APP_TUNING } app_mode_t;
typedef enum { PHASE_P0, PHASE_P1, PHASE_P2, PHASE_P3, PHASE_ALL_RED } app_phase_t;
typedef enum { APP_RED, APP_GREEN, APP_AMBER } app_light_t;
typedef enum { APP_EVENT_NONE, APP_EVENT_PRESS, APP_EVENT_LONG } app_event_t;
typedef enum { PARAM_RED, PARAM_GREEN, PARAM_AMBER } app_parameter_t;
typedef enum { TUNING_EDITING, TUNING_SAVED, TUNING_INVALID } app_tuning_status_t;
typedef struct { uint16_t red_s, green_s, amber_s; } timing_config_t;
typedef struct {
    app_mode_t mode;
    app_phase_t phase;
    timing_config_t committed, draft;
    app_parameter_t selected;
    app_tuning_status_t tuning_status;
    uint32_t phase_remaining_ms;
    bool countdown_available;
    bool fault;
} app_snapshot_t;

extern app_snapshot_t app_state;
extern timing_config_t app_timing;
bool timing_validate(const timing_config_t *cfg);
#endif
