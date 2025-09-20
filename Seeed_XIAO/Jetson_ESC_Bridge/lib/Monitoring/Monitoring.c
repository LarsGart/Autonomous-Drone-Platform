#include "Monitoring.h"

// Initialize the Monitoring instance
Monitoring_t monitoring = {
  .battery_voltage_raw = 0,
  .jetson_current_raw = 0,
  .esc_current_raw = 0,
  .readings_valid = false
};

// ----------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------------
/*
  @name configure_pins
  @brief Configures the pins to be used as ADC inputs
*/
static void configure_pins(void) {
  /*
    Battery Voltage:  PB09 (AIN[3])
    Jetson Current:   PB08 (AIN[2])
    ESC Current:      PA09 (AIN[17])
  */
  // Enable pin mux
  PORT->Group[PORTB].PINCFG[PIN_PB09].bit.PMUXEN = 1;
  PORT->Group[PORTB].PINCFG[PIN_PB08].bit.PMUXEN = 1;
  PORT->Group[PORTA].PINCFG[PIN_PA09].bit.PMUXEN = 1;

  // Select pin mux function
  PORT->Group[PORTB].PMUX[4].bit.PMUXO = MUX_PB09B_ADC_AIN3;
  PORT->Group[PORTB].PMUX[4].bit.PMUXE = MUX_PB08B_ADC_AIN2;
  PORT->Group[PORTA].PMUX[4].bit.PMUXO = MUX_PA09B_ADC_AIN17;
}

/*
  @name configure_adc
  @brief Configures the ADC
*/
static void configure_adc(void) {
  /*
    Enable ADC clocks
  */
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;            // Enable APBC clock for ADC
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_ADC) |  // Select the ADC clock
                      GCLK_CLKCTRL_GEN_GCLK0 |    // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;         // Enable the clock
  /*
    Disable and reset the ADC
  */
  ADC->CTRLA.bit.ENABLE = 0;
  ADC->CTRLA.bit.SWRST = 1;
  while (ADC->CTRLA.bit.SWRST || ADC->STATUS.bit.SYNCBUSY);
}

// ----------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------
