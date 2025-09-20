#ifndef MONITORING_H
#define MONITORING_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint16_t battery_voltage_raw;
  uint16_t jetson_current_raw;
  uint16_t esc_current_raw;
  bool readings_valid;
} Monitoring_t;

// Global instance of Monitoring
extern Monitoring_t monitoring;

#ifdef __cplusplus
}
#endif

#endif