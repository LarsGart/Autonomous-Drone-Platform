#include <Arduino.h>

volatile uint8_t i = 0; // Global variable for SPI data reception
volatile uint8_t j = 0; // Global variable for SPI data transmission
volatile bool rx_done; // Flag to indicate reception completion
volatile uint8_t num_bytes = 0; // Number of bytes received

volatile bool dre_interrupt = false; // Flag to indicate Data Register Empty interrupt
volatile bool ssl_interrupt = false; // Flag to indicate Slave Select interrupt
volatile bool txc_interrupt = false; // Flag to indicate Transmit Complete interrupt
volatile bool rxc_interrupt = false; // Flag to indicate Receive Complete interrupt

uint8_t tx_buffer[4] = {0x01, 0x02, 0x03, 0x04}; // Buffer for SPI transmission
volatile uint8_t rx_buffer[16]; // Buffer for SPI reception

void SPI_Slave_Init() {
  /*
    Initialize pins as SERCOM0 pins
  */
  // Enable pin mux
  PORT->Group[PORTA].PINCFG[PIN_PA08].reg = PORT_PINCFG_PMUXEN;
  PORT->Group[PORTA].PINCFG[PIN_PA09].reg = PORT_PINCFG_PMUXEN;
  PORT->Group[PORTA].PINCFG[PIN_PA10].reg = PORT_PINCFG_PMUXEN;
  PORT->Group[PORTA].PINCFG[PIN_PA11].reg = PORT_PINCFG_PMUXEN;

  // Select pin mux function
  PORT->Group[PORTA].PMUX[4].bit.PMUXE = MUX_PA08C_SERCOM0_PAD0;
  PORT->Group[PORTA].PMUX[4].bit.PMUXO = MUX_PA09C_SERCOM0_PAD1;
  PORT->Group[PORTA].PMUX[5].bit.PMUXE = MUX_PA10C_SERCOM0_PAD2;
  PORT->Group[PORTA].PMUX[5].bit.PMUXO = MUX_PA11C_SERCOM0_PAD3;

  /*
    Set up SERCOM0 interrupt
  */
  // Disable SERCOM0 interrupt
  NVIC_DisableIRQ(SERCOM0_IRQn);

  // Clear pending interrupt
  NVIC_ClearPendingIRQ(SERCOM0_IRQn);

  // Set interrupt priority
  NVIC_SetPriority(SERCOM0_IRQn, 2);

  /*
    Initialize SERCOM0
  */
  // Disable SERCOM0
  SERCOM0->SPI.CTRLA.bit.ENABLE = 0;

  // Wait for disable synchronization
  while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE);
  
  // Software reset SERCOM0
  SERCOM0->SPI.CTRLA.bit.SWRST = 1;
  
  // Wait for reset and synchronization
  while (SERCOM0->SPI.CTRLA.bit.SWRST || SERCOM0->SPI.SYNCBUSY.bit.SWRST);

  /*
    Initialize SERCOM0 clock
  */
  // Enable APBC clock for SERCOM0
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

  // Set up clock to SERCOM0
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM0_CORE) |  // Select the SERCOM0 core clock
                      GCLK_CLKCTRL_GEN_GCLK0 |                 // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;                      // Enable the clock
  
  // Wait for clock register synchronization
  while (GCLK->STATUS.bit.SYNCBUSY);
  
  // Set up SPI control A register
  SERCOM0->SPI.CTRLA.bit.DOPO = 0x2; // DATA PAD2 is used as slave output: MISO
  SERCOM0->SPI.CTRLA.bit.MODE = 0x2; // SPI slave operation
  SERCOM0->SPI.CTRLA.bit.IBON = 0x1; // Immediate Buffer Overflow Notification
  SERCOM0->SPI.CTRLA.bit.RUNSTDBY = 1; // Wake on Receive Complete interrupt

  // Enable receiver
  SERCOM0->SPI.CTRLB.bit.RXEN = 1;

  // Wait for CTRLB register synchronization
  while (SERCOM0->SPI.SYNCBUSY.bit.CTRLB);

  // Set up SERCOM0 interrupt
  SERCOM0->SPI.INTENSET.bit.RXC = 0x1; // Enable Receive Complete interrupt
  SERCOM0->SPI.INTENSET.bit.TXC = 0x1; // Enable Transmit Complete interrupt
  SERCOM0->SPI.INTENSET.bit.DRE = 0x1; // Enable Data Register Empty interrupt

  // Enable SERCOM0
  SERCOM0->SPI.CTRLA.bit.ENABLE = 1;

  // Wait for enable synchronization
  while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE);

  /*
    Preload DATA register
  */
  // Preload the DATA register with the first byte to be sent
  SERCOM0->SPI.DATA.reg = tx_buffer[0]; // Load the first byte to be sent

  /*
    Enable SERCOM0 interrupt
  */
  NVIC_EnableIRQ(SERCOM0_IRQn);
}

void setup() {
  SPI_Slave_Init();
  /*
    Enable Serial
  */
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect

  Serial.println("Setup complete");
}

void loop() {
  if (rx_done) { // Check if SPI reception is complete
    Serial.printf("Received %d bytes: ", num_bytes); // Print the number of received bytes
    for (uint8_t k = 0; k < num_bytes; k++) {
      Serial.print(rx_buffer[k], HEX); // Print received data in hexadecimal format
      Serial.print(" ");
    }
    Serial.println();
    rx_done = false; // Reset the reception flag
  }

  if (dre_interrupt) { // Check if Data Register Empty interrupt occurred
    Serial.println("Data Register Empty interrupt occurred");
    dre_interrupt = false; // Reset the interrupt flag
  }
  if (ssl_interrupt) { // Check if Slave Select Low interrupt occurred
    Serial.println("Slave Select Low interrupt occurred");
    ssl_interrupt = false; // Reset the interrupt flag
  }
  if (txc_interrupt) { // Check if Transmit Complete interrupt occurred
    Serial.println("Transmit Complete interrupt occurred");
    txc_interrupt = false; // Reset the interrupt flag
  }
  if (rxc_interrupt) { // Check if Receive Complete interrupt occurred
    Serial.println("Receive Complete interrupt occurred");
    rxc_interrupt = false; // Reset the interrupt flag
  }
}

void SERCOM0_Handler() {
  if (SERCOM0->SPI.INTFLAG.bit.DRE) {
    dre_interrupt = true; // Set Data Register Empty interrupt flag
    SERCOM0->SPI.DATA.reg = tx_buffer[j++]; // Increment j for next data to be sent
    if (j == 4) { // Reset j to 0 after sending 4 bytes
      j = 0;
    }
  }

  if (SERCOM0->SPI.INTFLAG.bit.RXC) {
    rxc_interrupt = true; // Set Receive Complete interrupt flag
    rx_buffer[i++] = SERCOM0->SPI.DATA.reg; // Read data from the master and store it in the rx_buffer
  }

  if (SERCOM0->SPI.INTFLAG.bit.TXC) {
    txc_interrupt = true; // Set Transmit Complete interrupt
    rx_done = true; // Set the reception flag to true when transmission is complete
    num_bytes = i; // Store the number of bytes received
    i = 0; // Reset i for the next reception
    SERCOM0->SPI.INTFLAG.bit.TXC = 1; // Clear Transmit Complete interrupt
  }
}