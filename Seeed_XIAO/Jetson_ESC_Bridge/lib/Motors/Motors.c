#include "Motors.h"

#define PWM_FREQUENCY 50 // 50Hz for standard ESCs

#define PWM_PERIOD (48000000 / 16 / PWM_FREQUENCY) // Timer period for 50Hz with prescaler 16
#define CLKS_PER_MICROSECOND (48000000 / 16 / 1000000) // Clock cycles per microsecond

#define CONVERT_SPEED_TO_CC(speed) (CLKS_PER_MICROSECOND * (int)(0.4885 * speed + 1000)) // Convert speed (0-2047) to timer ticks

// Initialize the Motors instance
Motors_t motors = {
  .armed = false
};

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
  // Disable pin mux
  PORT->Group[PORTA].PINCFG[PIN_PA02].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[PIN_PA08].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[PIN_PA10].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[PIN_PA11].bit.PMUXEN = 0;

  // Set pin directions to output
  PORT->Group[PORTA].DIRSET.reg = PORT_PA02 |
                                  PORT_PA08 |
                                  PORT_PA10 |
                                  PORT_PA11;
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
  while (GCLK->STATUS.bit.SYNCBUSY);
  /*
    Reset TCC0 and wait for synchronization
  */
  TCC0->CTRLA.bit.SWRST = 1;
  while (TCC0->CTRLA.bit.SWRST || TCC0->SYNCBUSY.bit.SWRST);
  /*
    Configure TCC0 for PWM generation
  */
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV16;  // Set prescaler to 16
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;       // Normal PWM mode
  TCC0->PER.reg = PWM_PERIOD;                   // Set period for desired frequency
  while (TCC0->SYNCBUSY.bit.PER || TCC0->SYNCBUSY.bit.WAVE);
  /*
    Set initial duty cycles to 1000 microseconds (motors off)
  */
  TCC0->CC[0].reg = CONVERT_SPEED_TO_CC(0); // Motor 1
  TCC0->CC[1].reg = CONVERT_SPEED_TO_CC(0); // Motor 2
  TCC0->CC[2].reg = CONVERT_SPEED_TO_CC(0); // Motor 3
  TCC0->CC[3].reg = CONVERT_SPEED_TO_CC(0); // Motor 4
  while (TCC0->SYNCBUSY.bit.CC0 || TCC0->SYNCBUSY.bit.CC1 || TCC0->SYNCBUSY.bit.CC2 || TCC0->SYNCBUSY.bit.CC3);
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
  while (TCC0->SYNCBUSY.bit.ENABLE);
}

// ----------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------
/*
  @name motors_init
  @brief Initializes motor control by configuring pins and timers
*/
void motors_init(void) {
  NVIC_DisableIRQ(TCC0_IRQn);
  NVIC_ClearPendingIRQ(TCC0_IRQn);
  NVIC_SetPriority(TCC0_IRQn, 1);

  configure_pins();
  configure_timers();

  NVIC_EnableIRQ(TCC0_IRQn);
}

/*
  @name update_motor_pulse_widths
  @brief Updates the pulse widths for all motors
  @param speeds Array of speed values for motors 0-3 (0-2047)
*/
void update_motor_pulse_widths(const uint16_t* speeds) {
  TCC0->CCB[0].reg = CONVERT_SPEED_TO_CC(speeds[0]);
  TCC0->CCB[1].reg = CONVERT_SPEED_TO_CC(speeds[1]);
  TCC0->CCB[2].reg = CONVERT_SPEED_TO_CC(speeds[2]);
  TCC0->CCB[3].reg = CONVERT_SPEED_TO_CC(speeds[3]);
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