/*
  Motors.h - Header file for motor control functions
*/

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool armed;
} Motors_t;

// Global instance of Motors
extern Motors_t motors;

// Functions
void motors_init(void);
void update_motor_pulse_widths(const uint16_t* pulse_widths);

#ifdef __cplusplus
}
#endif

#endif