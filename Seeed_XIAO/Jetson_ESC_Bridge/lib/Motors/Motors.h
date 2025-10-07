/*!
  \file Motors.h
  \brief 4-in-1 ESC interface

  This library uses TCC0 to create four 50Hz PWM outputs to control the
  four motors for the drone

  \todo Update this library to use/support DSHOT
*/
#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool armed;
  bool debug_mode;
} Motors_t;

// Global instance of Motors
extern Motors_t motors;

// Functions
void motors_init(void);
void set_motor_speed(const uint16_t* speeds);

#ifdef __cplusplus
}
#endif

#endif