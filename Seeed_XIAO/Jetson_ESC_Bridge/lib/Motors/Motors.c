#include "Motors.h"

#define TC_CYCLES_PER_BIT 3     // Number of timer cycles per DShot bit
#define DSHOT_BAUD        300E3 // Dshot baudrate in bits/s
#define NUM_DSHOT_BITS    16    // Number of bits in a DShot packet
#define SPEED_CMD_BASE    48    // Command value for a motor speed of 0

// Pin number of each motor
#define MOTOR_1_PIN 8
#define MOTOR_2_PIN 11
#define MOTOR_3_PIN 10
#define MOTOR_4_PIN 2

// Constants
static const uint8_t MOTOR_BIT_POSITIONS[4] = { // Bit positions to be used for writing the OUTTGL register
  MOTOR_1_PIN,
  MOTOR_2_PIN,
  MOTOR_3_PIN,
  MOTOR_4_PIN
};
static const uint8_t TOGGLE_SEQUENCES[2][TC_CYCLES_PER_BIT] = { // Pin toggle sequence for each DShot bit
  {1, 1, 0},
  {1, 0, 1}
};
static const uint32_t ALL_MOTOR_BITS =  (1 << MOTOR_1_PIN) |
                                        (1 << MOTOR_2_PIN) |
                                        (1 << MOTOR_3_PIN) | 
                                        (1 << MOTOR_4_PIN);
static const uint8_t NUM_TOGGLE_WORDS = NUM_DSHOT_BITS * TC_CYCLES_PER_BIT;
static const uint16_t TC4_CC_VAL = (uint16_t) (F_CPU / (TC_CYCLES_PER_BIT * DSHOT_BAUD));

// Variables
static uint8_t dshot_data_bits[4][NUM_DSHOT_BITS];
static bool dshot_data_bits_valid[4];
static uint32_t toggle_data[NUM_TOGGLE_WORDS];
static uint8_t toggle_data_i = 0;
static bool toggle_data_valid = false;

// Initialize the Motors instance
Motors_t motors = {
  .armed = false,
  .debug_mode = false
};

// ----------------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------------
/*!
  \brief Convert data into an array of DShot bits and update the dshot_data_bits array
  \param data Command being sent to a motor (0-2047)
  \param motor Index for the target motor (0-3)
*/
static void generate_dshot_bitstream(uint16_t data, uint8_t motor) {
  /*
    The data needs to be left-shifted by 1 first to account for the telemetry bit before
    the CRC calculation can be performed
  */
  uint16_t shifted_data = data << 1;
  uint16_t crc = (shifted_data ^ (shifted_data >> 4) ^ (shifted_data >> 8)) & 0x0F;
  uint16_t dshot_data = (shifted_data << 4) | crc;
  for (int i = 0; i < NUM_DSHOT_BITS; i++) {
    dshot_data_bits[motor][i] = (dshot_data >> i) & 0x01;
  }
  dshot_data_bits_valid[motor] = true;
}

/*!
  \brief Update the toggle register data array based on the DShot data for each valid motor
*/
static void update_toggle_data_array(void) {
  // Clear the toggle data array
  for (int i = 0; i < NUM_TOGGLE_WORDS; i++) {
    toggle_data[i] = 0;
  };

  // Update the toggle data array based on the valid DShot data for each motor
  for (int m = 0; m < 4; m++) {
    if (!dshot_data_bits_valid[m])
      continue;
    dshot_data_bits_valid[m] = false;

    uint8_t motor_bit_pos = MOTOR_BIT_POSITIONS[m];
    for (int i = 0; i < NUM_DSHOT_BITS; i++) {
      uint8_t toggle_data_base_i = TC_CYCLES_PER_BIT * i;
      uint8_t dshot_data_bit = dshot_data_bits[m][i];
      uint8_t *bit_toggle_sequence = TOGGLE_SEQUENCES[dshot_data_bit];
      for (int t = 0; t < TC_CYCLES_PER_BIT; t++) {
        toggle_data[toggle_data_base_i + t] |= (bit_toggle_sequence[t] << motor_bit_pos);
      }
    }
  }
  toggle_data_i = 0;
  toggle_data_valid = true;
}

/*!
  \brief Configure pins to be used for DShot output

  Motor 1: PA8
  Motor 2: PA11
  Motor 3: PA10
  Motor 4: PA2
*/
static void configure_pins(void) {
  // Disable pin mux
  PORT->Group[PORTA].PINCFG[MOTOR_1_PIN].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[MOTOR_2_PIN].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[MOTOR_3_PIN].bit.PMUXEN = 0;
  PORT->Group[PORTA].PINCFG[MOTOR_4_PIN].bit.PMUXEN = 0;

  // Set pin directions to output and set them low
  PORT->Group[PORTA].DIRSET.reg = ALL_MOTOR_BITS;
  PORT->Group[PORTA].OUTCLR.reg = ALL_MOTOR_BITS;
}

/*!
  \brief Configure TC4 to overflow at 3x the DShot baudrate
*/
static void configure_timer(void) {
  // Enable TC4 clocks
  PM->APBCMASK.reg |= PM_APBCMASK_TC4;                // Enable APBC clock for TC4
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_TC4_TC5) |  // Select the TC4 and TC5 clock
                      GCLK_CLKCTRL_GEN_GCLK0 |        // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;             // Enable the clock
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Reset TC4 and wait for synchronization
  TC4->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
  TC4->COUNT16.CTRLA.bit.SWRST = 1;
  while (TC4->COUNT16.CTRLA.bit.SWRST || TC4->COUNT16.STATUS.bit.SYNCBUSY);

  // Configure TC4 counting frequency
  TC4->COUNT16.CTRLA.reg =  TC_CTRLA_WAVEGEN_MFRQ | // Match Frequency Generation to control period with CC
                            TC_CTRLA_MODE_COUNT16;  // Use 16-bit counter mode
  TC4->COUNT16.CC->reg = TC4_CC_VAL;                // Set CC to give 3x baudrate frequency
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

  // Enable TC4 overflow interrupt
  TC4->COUNT16.INTENSET.bit.OVF = 1;

  // Enable TC4
  TC4->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
}

// ----------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------
/*!
  \brief Initialize motor control by configuring pins and timer
*/
void motors_init(void) {
  NVIC_DisableIRQ(TC4_IRQn);
  NVIC_ClearPendingIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0);

  configure_pins();
  configure_timer();

  NVIC_EnableIRQ(TC4_IRQn);
}

/*!
  \brief Send a DShot command to a motor
  \param cmd Command to be sent (0-2047)
  \param motor Index of the target motor (0-3)
*/
void send_motor_cmd(const uint16_t cmd, const uint8_t motor) {
  // Don't update the DShot bitstream if we're currently sending DShot data
  if (toggle_data_valid)
    return;
  toggle_data_valid = true;
  generate_dshot_bitstream(cmd, motor);
  update_toggle_data_array();
}

/*!
  \brief Update the pulse widths for all motors
  \param speeds Array of 4 speed values (0-1999) for motors
*/
void send_motor_speeds(const uint16_t* speeds) {
  // Don't update the DShot bitstream if we're currently sending DShot data
  if (toggle_data_valid)
    return;
  toggle_data_valid = true;
  for (int m = 0; m < 4; m++) {
    generate_dshot_bitstream(speeds[m] + SPEED_CMD_BASE, m);
  }
  update_toggle_data_array();
}
// ----------------------------------------------------------------------
// INTERRUPT HANDLER
// ----------------------------------------------------------------------
void TC4_Handler(void) {
  // Write the output toggle register on Overflow interrupt
  if (TC4->COUNT16.INTFLAG.bit.OVF) {
    TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
    if (toggle_data_valid) {
      PORT->Group[PORTA].OUTTGL.reg = toggle_data[toggle_data_i++];
      if (toggle_data_i == NUM_TOGGLE_WORDS) {
        toggle_data_valid = false;
      }
    }
  }
}