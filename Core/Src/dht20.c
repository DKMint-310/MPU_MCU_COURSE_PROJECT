#include "dht20.h"

#include <stddef.h>

enum {
    POWER_UP_MS = 100,
    SAMPLE_INTERVAL_MS = 2000,
    CONVERSION_MS = 80,
    CONVERSION_TIMEOUT_MS = 150,
    STATUS_READY = 0x18,
    STATUS_BUSY = 0x80
};

typedef enum {
    CHECK_STATUS,
    RESET_WRITE,
    RESET_READ,
    RESET_RESTORE,
    START_CONVERSION,
    READ_FRAME
} dht20_state_t;

static const uint8_t reset_registers[] = {0x1B, 0x1C, 0x1E};
static peripheral_i2c_t transport;
static env_sample_t latest;
static dht20_state_t state;
static uint8_t reset_index;
static uint8_t reset_data[3];
static uint32_t started_at;
static uint32_t wait_started;
static uint32_t wait_ms;

static void schedule(dht20_state_t next, uint32_t delay)
{
    state = next;
    wait_started = transport.now_ms();
    wait_ms = delay;
}

bool dht20_init(const peripheral_i2c_t *bus)
{
    if (bus == NULL || bus->write == NULL || bus->read == NULL ||
        bus->now_ms == NULL) {
        return false;
    }

    transport = *bus;
    latest = (env_sample_t){0};
    schedule(CHECK_STATUS, POWER_UP_MS);
    return true;
}

static bool sensor_write(const uint8_t *data, uint16_t length)
{
    return transport.write(transport.context, DHT20_ADDRESS, data, length);
}

static bool sensor_read(uint8_t *data, uint16_t length)
{
    return transport.read(transport.context, DHT20_ADDRESS, data, length);
}

static void retry_later(void)
{
    latest.stale = latest.valid;
    schedule(CHECK_STATUS, SAMPLE_INTERVAL_MS);
}

static uint8_t frame_crc(const uint8_t frame[7])
{
    uint8_t crc = 0xFF;
    for (uint8_t byte = 0; byte < 6; ++byte) {
        crc ^= frame[byte];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80U) ? 0x31U : 0U));
        }
    }
    return crc;
}

static void read_measurement(void)
{
    uint8_t frame[7];
    if (!sensor_read(frame, sizeof frame)) {
        retry_later();
        return;
    }
    if ((frame[0] & STATUS_BUSY) != 0U) {
        if (transport.now_ms() - started_at >= CONVERSION_TIMEOUT_MS) {
            retry_later();
        }
        return;
    }
    if ((frame[0] & STATUS_READY) != STATUS_READY || frame_crc(frame) != frame[6]) {
        retry_later();
        return;
    }

    uint32_t humidity = ((uint32_t)frame[1] << 12) |
                        ((uint32_t)frame[2] << 4) | (frame[3] >> 4);
    uint32_t temperature = ((uint32_t)(frame[3] & 0x0FU) << 16) |
                           ((uint32_t)frame[4] << 8) | frame[5];
    uint32_t now = transport.now_ms();
    latest = (env_sample_t){
        .temperature_c = temperature * (200.0f / 1048576.0f) - 50.0f,
        .humidity_pct = humidity * (100.0f / 1048576.0f),
        .valid = true,
        .stale = false,
        .sample_tick = now
    };
    uint32_t elapsed = now - started_at;
    schedule(CHECK_STATUS,
             elapsed < SAMPLE_INTERVAL_MS ? SAMPLE_INTERVAL_MS - elapsed : 0U);
}

void dht20_task(void)
{
    if (transport.now_ms == NULL || transport.now_ms() - wait_started < wait_ms) {
        return;
    }

    switch (state) {
    case CHECK_STATUS: {
        uint8_t status;
        if (!sensor_read(&status, 1) || (status & STATUS_BUSY) != 0U) {
            retry_later();
        } else if ((status & STATUS_READY) == STATUS_READY) {
            schedule(START_CONVERSION, 10);
        } else {
            reset_index = 0;
            schedule(RESET_WRITE, 0);
        }
        break;
    }
    case RESET_WRITE: {
        const uint8_t command[] = {reset_registers[reset_index], 0, 0};
        if (sensor_write(command, sizeof command)) {
            schedule(RESET_READ, 20);
        } else {
            retry_later();
        }
        break;
    }
    case RESET_READ:
        if (sensor_read(reset_data, sizeof reset_data)) {
            reset_data[0] = (uint8_t)(0xB0U | reset_registers[reset_index]);
            schedule(RESET_RESTORE, 10);
        } else {
            retry_later();
        }
        break;
    case RESET_RESTORE:
        if (!sensor_write(reset_data, sizeof reset_data)) {
            retry_later();
        } else if (++reset_index == sizeof reset_registers) {
            schedule(START_CONVERSION, 10);
        } else {
            schedule(RESET_WRITE, 10);
        }
        break;
    case START_CONVERSION: {
        const uint8_t command[] = {0xAC, 0x33, 0x00};
        if (sensor_write(command, sizeof command)) {
            schedule(READ_FRAME, CONVERSION_MS);
            started_at = wait_started;
        } else {
            retry_later();
        }
        break;
    }
    case READ_FRAME:
        read_measurement();
        break;
    }
}

env_sample_t dht20_get_latest(void)
{
    return latest;
}
