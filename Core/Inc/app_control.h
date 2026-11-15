#ifndef APP_CONTROL_H
#define APP_CONTROL_H
#include "global.h"

typedef struct {
    app_event_t (*button_take_event)(uint8_t id);
    bool (*timer_start)(uint8_t id, uint32_t ms);
    void (*timer_cancel)(uint8_t id);
    bool (*timer_take_expired)(uint8_t id);
    bool (*traffic_apply_pair)(app_light_t road1, app_light_t road2);
    void (*traffic_all_red)(void);

    uint32_t (*timer_remaining_ms)(uint8_t id);
} app_services_t;

bool app_control_init(const app_services_t *services);
app_snapshot_t app_get_snapshot(void);
#endif
