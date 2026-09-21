#include "sensor.h"

static const char *recent_sensors[SENSOR_RECENT_COUNT];
static size_t recent_count = 0;

void sensor_init(void) {
    recent_count = 0;

    for (size_t i = 0; i < SENSOR_RECENT_COUNT; ++i) {
        recent_sensors[i] = "";
    }
}

void sensor_poll(void) {
    /*
     * TODO for hardware version:
     *
     * Poll CAN bus messages related to sensors.
     * When a sensor is triggered, call sensor_record_trigger().
     */
}

void sensor_record_trigger(const char *sensor_name) {
    if (sensor_name == 0) {
        return;
    }

    for (size_t i = SENSOR_RECENT_COUNT - 1; i > 0; --i) {
        recent_sensors[i] = recent_sensors[i - 1];
    }

    recent_sensors[0] = sensor_name;

    if (recent_count < SENSOR_RECENT_COUNT) {
        recent_count += 1;
    }
}

size_t sensor_recent_count(void) {
    return recent_count;
}

const char *sensor_recent(size_t index) {
    if (index >= recent_count || index >= SENSOR_RECENT_COUNT) {
        return "";
    }

    return recent_sensors[index];
}
