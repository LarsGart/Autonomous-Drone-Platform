#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool active;
} Status_LED_t;

// Global instance of Status_LED
extern Status_LED_t status_led;

// Functions
void status_led_init(void);
void set_led_state(void);

#ifdef __cplusplus
}
#endif

#endif;