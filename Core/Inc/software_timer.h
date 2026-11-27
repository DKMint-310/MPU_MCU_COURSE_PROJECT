#ifndef SOFTWARE_TIMER_H
#define SOFTWARE_TIMER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TIMER_COUNT     8U
#define TIMER_TICK_MS   10U

void timer_init(void);
bool timer_start(uint8_t id, uint32_t ms);
void timer_cancel(uint8_t id);
bool timer_take_expired(uint8_t id);
uint32_t timer_remaining_ms(uint8_t id);
void timer_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* SOFTWARE_TIMER_H */
