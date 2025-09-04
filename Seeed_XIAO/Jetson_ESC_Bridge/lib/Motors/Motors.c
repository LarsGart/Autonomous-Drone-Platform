#include "Motors.h"

#define PWM_FREQUENCY 50 // 50Hz for standard ESCs

#define PWM_PERIOD (48000000 / 16 / PWM_FREQUENCY) // Timer period for 50Hz with prescaler 16
#define CLKS_PER_MICROSECOND (48000000 / 16 / 1000000) // Clock cycles per microsecond

#define CONVERT_MICROS_TO_DUTY(micros) (CLKS_PER_MICROSECOND * micros) // Convert microseconds to timer ticks

// Initialize the Motors instance
Motors_t motors = {
  .armed = false
};

// Error flag
static uint8_t error_flag = 0;

// ----------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------------
/*
  @name configure_pins
  @brief Configure pins for motor control
*/
static void configure_pins(void) {
  /*
    Motor 1: PA8
    Motor 2: PA11
    Motor 3: PA10
    Motor 4: PA2
  */
  // Set pin directions to output
  PORT->Group[PORTA].DIRSET.reg = PORT_PA02 |
                                  PORT_PA08 |
                                  PORT_PA10 |
                                  PORT_PA11;
  // Disable pin mux
  PORT->Group[PORTA].PINCFG[PIN_PA02].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[PIN_PA08].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[PIN_PA10].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[PIN_PA11].bit.PMUXEN = 0;
}

/*
  @name configure_timers
  @brief Configure TCC0 for four 50Hz PWM outputs
*/
static void configure_timers(void) {
  /*
    Enable TCC0 clocks
  */
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0;                 // Enable APBC clock for TCC0
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_TCC0_TCC1) |  // Select the TCC0 and TCC1 clock
                      GCLK_CLKCTRL_GEN_GCLK0 |          // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;               // Enable the clock

  uint32_t timeout = 100000; // Timeout counter to prevent infinite loop
  while (GCLK->STATUS.bit.SYNCBUSY && --timeout);
  if (timeout == 0) {
    error_flag = 1; // Indicate timeout error
    return;
  }
  /*
    Reset TCC0 and wait for synchronization
  */
  TCC0->CTRLA.bit.SWRST = 1;
  timeout = 100000;
  while ((TCC0->CTRLA.bit.SWRST || TCC0->SYNCBUSY.bit.SWRST) && --timeout);
  if (timeout == 0) {
    error_flag = 2; // Indicate timeout error
    return;
  }
  /*
    Configure TCC0 for PWM generation
  */
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV16;  // Set prescaler to 16
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;       // Normal PWM mode
  TCC0->PER.reg = PWM_PERIOD;                   // Set period for desired frequency

  timeout = 100000;
  while ((TCC0->SYNCBUSY.bit.PER || TCC0->SYNCBUSY.bit.WAVE) && --timeout);
  if (timeout == 0) {
    error_flag = 3; // Indicate timeout error
    return;
  }
  /*
    Set initial duty cycles to 1000 microseconds (motors off)
  */
  TCC0->CC[0].reg = CONVERT_MICROS_TO_DUTY(1000); // Motor 1
  TCC0->CC[1].reg = CONVERT_MICROS_TO_DUTY(1000); // Motor 2
  TCC0->CC[2].reg = CONVERT_MICROS_TO_DUTY(1000); // Motor 3
  TCC0->CC[3].reg = CONVERT_MICROS_TO_DUTY(1000); // Motor 4

  timeout = 100000;
  while ((TCC0->SYNCBUSY.bit.CC0 || TCC0->SYNCBUSY.bit.CC1 || TCC0->SYNCBUSY.bit.CC2 || TCC0->SYNCBUSY.bit.CC3) && --timeout);
  if (timeout == 0) {
    error_flag = 4; // Indicate timeout error
    return;
  }
  /*
    Enable TCC0 interrupts for overflow and match/capture channels
  */
  TCC0->INTENSET.reg =  TCC_INTENSET_MC0 |  // Compare/Capture Channel 0
                        TCC_INTENSET_MC1 |  // Compare/Capture Channel 1
                        TCC_INTENSET_MC2 |  // Compare/Capture Channel 2
                        TCC_INTENSET_MC3 |  // Compare/Capture Channel 3
                        TCC_INTENSET_OVF;   // Overflow
  /*
    Enable TCC0
  */
  TCC0->CTRLA.bit.ENABLE = 1;

  timeout = 100000;
  while (TCC0->SYNCBUSY.bit.ENABLE && --timeout);
  if (timeout == 0) {
    error_flag = 5; // Indicate timeout error
    return;
  }
}

// ----------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------
/*
  @name motors_init
  @brief Initializes motor control by configuring pins and timers
  @return 0 if successful, non-zero error code otherwise
*/
uint8_t motors_init(void) {
  NVIC_DisableIRQ(TCC0_IRQn);
  NVIC_ClearPendingIRQ(TCC0_IRQn);
  NVIC_SetPriority(TCC0_IRQn, 1);

  configure_pins();
  configure_timers();

  NVIC_EnableIRQ(TCC0_IRQn);

  return error_flag; // Return 0 if successful, non-zero for errors
}

/*
  @name set_motor_pulse_width
  @brief Sets the pulse width for a specific motor (in microseconds)
  @param motor Motor index (0-3)
  @param pulse_width Pulse width in microseconds (1000-2000)
*/
void set_motor_pulse_width(uint8_t motor, uint16_t pulse_width) {
  if (motor > 3 || pulse_width < 1000 || pulse_width > 2000) {
    return; // Invalid motor index or pulse width
  }
  uint32_t compare_val = CONVERT_MICROS_TO_DUTY(pulse_width);
  switch (motor) {
    case 0:
      TCC0->CCB[0].reg = compare_val;
      break;
    case 1:
      TCC0->CCB[1].reg = compare_val;
      break;
    case 2:
      TCC0->CCB[2].reg = compare_val;
      break;
    case 3:
      TCC0->CCB[3].reg = compare_val;
      break;
  }
}
// ----------------------------------------------------------------------
// INTERRUPT HANDLER
// ----------------------------------------------------------------------
void TCC0_Handler(void) {
  // Set motor 1 pin low (PA08) on Compare/Capture Channel 0 interrupt
  if (TCC0->INTFLAG.bit.MC0) {
    TCC0->INTFLAG.reg = TCC_INTFLAG_MC0;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA08;
  }

  // Set motor 2 pin low (PA11) on Compare/Capture Channel 1 interrupt
  if (TCC0->INTFLAG.bit.MC1) {
    TCC0->INTFLAG.reg = TCC_INTFLAG_MC1;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA11;
  }

  // Set motor 3 pin low (PA10) on Compare/Capture Channel 2 interrupt
  if (TCC0->INTFLAG.bit.MC2) {
    TCC0->INTFLAG.reg = TCC_INTFLAG_MC2;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA10;
  }

  // Set motor 4 pin low (PA02) on Compare/Capture Channel 3 interrupt
  if (TCC0->INTFLAG.bit.MC3) {
    TCC0->INTFLAG.reg = TCC_INTFLAG_MC3;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA02;
  }

  // Set all motor pins high at the start of the next PWM cycle if armed
  if (TCC0->INTFLAG.bit.OVF) {
    TCC0->INTFLAG.reg = TCC_INTFLAG_OVF;
    if (motors.armed) {
      PORT->Group[PORTA].OUTSET.reg = PORT_PA02 |
                                      PORT_PA08 |
                                      PORT_PA10 |
                                      PORT_PA11; 
    }
  }
}