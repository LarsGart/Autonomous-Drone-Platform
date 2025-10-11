#include "Motors.h"

#define PWM_FREQUENCY 50 // 50Hz for standard ESCs
#define CLK_PRESCALER 16 // Clock division factor for GCLK_TCC0
#define PWM_PERIOD (F_CPU / CLK_PRESCALER / PWM_FREQUENCY) // Timer period for 50Hz with prescaler 16

// Initialize the Motors instance
Motors_t motors = {
  .armed = false,
  .debug_mode = false
};

// Constants
static const int CLKS_PER_MICROSECOND = (F_CPU / CLK_PRESCALER / 1000000); // Clock cycles per microsecond
static const int CC_1000US = (F_CPU / CLK_PRESCALER / 1000); // TCC CC value for a 1000 us pulse width

// ----------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------------
/*!
  \brief Convert a speed value to timer compare/capture value
  \param speed Speed value (0-1999)
  \return Corresponding timer compare/capture value
*/
static uint32_t convert_speed_to_cc(int speed) {
  return CLKS_PER_MICROSECOND * (int) (0.5 * speed + 1000); // Convert speed (0-1999) to timer ticks
}

/*!
  \brief Configure pins to be used for PWM output

  Motor 1: PA8
  Motor 2: PA11
  Motor 3: PA10
  Motor 4: PA2
*/
static void configure_pins(void) {
  // Disable pin mux
  PORT->Group[PORTA].PINCFG[2].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[8].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[10].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[11].bit.PMUXEN = 0;

  // Set pin directions to output
  PORT->Group[PORTA].DIRSET.reg = PORT_PA02 |
                                  PORT_PA08 |
                                  PORT_PA10 |
                                  PORT_PA11;
}

/*!
  \brief Configure TCC0 for four unique 50Hz PWM outputs
*/
static void configure_timers(void) {
  // Enable TCC0 clocks
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0;                 // Enable APBC clock for TCC0
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_TCC0_TCC1) |  // Select the TCC0 and TCC1 clock
                      GCLK_CLKCTRL_GEN_GCLK0 |          // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;               // Enable the clock
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Reset TCC0 and wait for synchronization
  TCC0->CTRLA.bit.ENABLE = 0;
  while (TCC0->SYNCBUSY.bit.ENABLE);
  TCC0->CTRLA.bit.SWRST = 1;
  while (TCC0->CTRLA.bit.SWRST || TCC0->SYNCBUSY.bit.SWRST);

  // Configure TCC0 for PWM generation
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV16;  // Set prescaler to 16
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;       // Normal PWM mode
  TCC0->PER.reg = PWM_PERIOD;                   // Set period for desired frequency
  while (TCC0->SYNCBUSY.bit.PER || TCC0->SYNCBUSY.bit.WAVE);

  // Set initial duty cycles to 1000 microseconds (motors off)
  TCC0->CC[0].reg = CC_1000US; // Motor 1
  TCC0->CC[1].reg = CC_1000US; // Motor 2
  TCC0->CC[2].reg = CC_1000US; // Motor 3
  TCC0->CC[3].reg = CC_1000US; // Motor 4
  while (TCC0->SYNCBUSY.bit.CC0 || TCC0->SYNCBUSY.bit.CC1 || TCC0->SYNCBUSY.bit.CC2 || TCC0->SYNCBUSY.bit.CC3);

  // Enable TCC0 interrupts for overflow and match/capture channels
  TCC0->INTENSET.reg =  TCC_INTENSET_MC0 |  // Compare/Capture Channel 0
                        TCC_INTENSET_MC1 |  // Compare/Capture Channel 1
                        TCC_INTENSET_MC2 |  // Compare/Capture Channel 2
                        TCC_INTENSET_MC3 |  // Compare/Capture Channel 3
                        TCC_INTENSET_OVF;   // Overflow

  // Enable TCC0
  TCC0->CTRLA.bit.ENABLE = 1;
  while (TCC0->SYNCBUSY.bit.ENABLE);
}

// ----------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------
/*!
  \brief Initialize motor control by configuring pins and timers
*/
void motors_init(void) {
  NVIC_DisableIRQ(TCC0_IRQn);
  NVIC_ClearPendingIRQ(TCC0_IRQn);
  NVIC_SetPriority(TCC0_IRQn, 0);

  configure_pins();
  configure_timers();

  NVIC_EnableIRQ(TCC0_IRQn);
}

/*!
  \brief Update the pulse widths for all motors
  \param speeds Array of 4 speed values (0-1999) for motors
*/
void set_motor_speed(const uint16_t* speeds) {
  TCC0->CCB[0].reg = convert_speed_to_cc(speeds[0]);
  TCC0->CCB[1].reg = convert_speed_to_cc(speeds[1]);
  TCC0->CCB[2].reg = convert_speed_to_cc(speeds[2]);
  TCC0->CCB[3].reg = convert_speed_to_cc(speeds[3]);
}
// ----------------------------------------------------------------------
// INTERRUPT HANDLER
// ----------------------------------------------------------------------
void TCC0_Handler(void) {
  // Set motor 1 pin low (PA08) on Compare/Capture Channel 0 interrupt
  if (TCC0->INTFLAG.bit.MC0) {
    TCC0->INTFLAG.reg |= TCC_INTFLAG_MC0;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA08;
  }

  // Set motor 2 pin low (PA11) on Compare/Capture Channel 1 interrupt
  if (TCC0->INTFLAG.bit.MC1) {
    TCC0->INTFLAG.reg |= TCC_INTFLAG_MC1;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA11;
  }

  // Set motor 3 pin low (PA10) on Compare/Capture Channel 2 interrupt
  if (TCC0->INTFLAG.bit.MC2) {
    TCC0->INTFLAG.reg |= TCC_INTFLAG_MC2;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA10;
  }

  // Set motor 4 pin low (PA02) on Compare/Capture Channel 3 interrupt
  if (TCC0->INTFLAG.bit.MC3) {
    TCC0->INTFLAG.reg |= TCC_INTFLAG_MC3;
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA02;
  }

  // Set all motor pins high at the start of the next PWM cycle if armed and not in debug mode
  if (TCC0->INTFLAG.bit.OVF) {
    TCC0->INTFLAG.reg |= TCC_INTFLAG_OVF;
    if (motors.armed && !motors.debug_mode) {
      PORT->Group[PORTA].OUTSET.reg = PORT_PA02 |
                                      PORT_PA08 |
                                      PORT_PA10 |
                                      PORT_PA11; 
    }
  }
}