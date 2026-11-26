#ifndef DHT20_H
#define DHT20_H

#include "peripheral_i2c.h"

#define DHT20_ADDRESS 0x38U

typedef struct {
    float temperature_c;
    float humidity_pct;
    bool valid;
    bool stale;
    uint32_t sample_tick; /* Milliseconds, from the transport clock. */
} env_sample_t;

bool dht20_init(const peripheral_i2c_t *bus);
/* Schedule every 10 ms. Starts a measurement no more often than every 2 s. */
void dht20_task(void);
env_sample_t dht20_get_latest(void);

#endif
