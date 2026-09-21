#ifndef SENSOR_H
#define SENSOR_H

#include <stddef.h>

#define SENSOR_RECENT_COUNT 10

void sensor_init(void);
void sensor_poll(void);

void sensor_record_trigger(const char *sensor_name);
size_t sensor_recent_count(void);
const char *sensor_recent(size_t index);

#endif
