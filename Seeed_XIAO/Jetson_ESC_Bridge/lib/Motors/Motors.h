/*!
  \file Motors.h
  \brief 4-in-1 ESC interface using DShot

  The library uses TC4 for timing the Dshot bits and the timer period is set so that it runs
  at 3x the DShot bitrate (e.g. 900kHz for DShot300)

  This is done because the DShot protocol uses pulse length to determine a 1 or 0, with a 0
  being roughly 1/3 of the duty cycle and a 1 being roughly 2/3 of the duty cycle

  The timer overflows 3x per bit and the state of the motor pins is modified on each overflow
  to send a bit

  An example diagram of sending a 0 and then a 1 is illustrated below:

                           0            1
                           |            |
                      /----------\/-----------\
                      _____       _________
          Output: ____|   |_______|       |____
                  -----------------------------
                  |   |   |   |   |   |   |   |
  Overflow count: 0   1   2   3   4   5   6   7

  More information about the DShot protocol can be found here:
  https://brushlesswhoop.com/dshot-and-bidirectional-dshot/

  To minimize register writes on each timer overflow, the OUTTGL register is the only one
  written instead of the OUTSET and OUTCLR registers

  This library sends a DShot command/speed to the ESC only when the corresponding function
  is called
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
void send_motor_cmd(const uint16_t cmd, const uint8_t motor);
void send_motor_speeds(const uint16_t* speeds);

#ifdef __cplusplus
}
#endif

#endif