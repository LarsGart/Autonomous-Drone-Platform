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
uint8_t motors_init(void);
void set_motor_pulse_width(uint8_t motor, uint16_t pulse_width);

#ifdef __cplusplus
}
#endif

#endif