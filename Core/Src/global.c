#include "global.h"
#include <stddef.h>
app_snapshot_t app_state = { .mode = APP_AUTO, .phase = PHASE_ALL_RED };
timing_config_t app_timing = {7, 5, 2};
bool timing_validate(const timing_config_t *cfg)
{
    return cfg != NULL && cfg->green_s >= 1 && cfg->green_s <= 60
        && cfg->amber_s >= 1 && cfg->amber_s <= 10
        && cfg->red_s >= 2 && cfg->red_s <= 70
        && cfg->red_s == cfg->green_s + cfg->amber_s;
}
