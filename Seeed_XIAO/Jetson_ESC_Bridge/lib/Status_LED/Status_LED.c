#include "Status_LED.h"

// Initialize the Status LED instance
Status_LED_t status_led = {
  .active = false
};

// ----------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------
/*!
  @brief Initialize the yellow LED pin
*/
void status_led_init(void) {
  // Disable pinmux for status LED pin
  PORT->Group[PORTA].PINCFG[17].bit.PMUXEN = 0;

  // Set pin direction to output
  PORT->Group[PORTA].DIRSET.reg |= PORT_PA17;

  // Set LED to inactive initially
  PORT->Group[PORTA].OUTCLR.reg |= PORT_PA17;
}

/*!
  @brief Set the yellow LED state based on the active flag
*/
void set_led_state(void) {
  if (status_led.active) {
    PORT->Group[PORTA].OUTSET.reg |= PORT_PA17;
  }
  else {
    PORT->Group[PORTA].OUTCLR.reg |= PORT_PA17;
  }
}