#include "SPISlave.h"

SPISlave::SPISlave() {
  // Constructor implementation (if needed)
}

/*
  Initialize pins as SERCOM0 pins
*/
void SPISlave::configure_spi_pins() {
  // Set PA10 as input w/ pullup for Slave Select (SS)
  // PORT->Group[PORTA].PINCFG[PIN_PA10].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN; // Enable input and pull-up for SS (PAD2)
  // PORT->Group[PORTA].OUTSET.reg = (1 << PIN_PA10); // Set PA10 (SS) high to enable pull-up resistor

  // Enable pin mux
  PORT->Group[PORTA].PINCFG[PIN_PA04].bit.PMUXEN = 1;
  PORT->Group[PORTA].PINCFG[PIN_PA05].bit.PMUXEN = 1;
  PORT->Group[PORTA].PINCFG[PIN_PA06].bit.PMUXEN = 1;
  PORT->Group[PORTA].PINCFG[PIN_PA07].bit.PMUXEN = 1;

  // Select pin mux function
  PORT->Group[PORTA].PMUX[2].bit.PMUXE = MUX_PA04D_SERCOM0_PAD0;
  PORT->Group[PORTA].PMUX[2].bit.PMUXO = MUX_PA05D_SERCOM0_PAD1;
  PORT->Group[PORTA].PMUX[3].bit.PMUXE = MUX_PA06D_SERCOM0_PAD2;
  PORT->Group[PORTA].PMUX[3].bit.PMUXO = MUX_PA07D_SERCOM0_PAD3;
}

/*
  Intialize SERCOM0 interrupt vector
*/
void SPISlave::initialize_sercom_irq() {
  // Disable SERCOM0 interrupt
  NVIC_DisableIRQ(SERCOM0_IRQn);

  // Clear pending interrupt
  NVIC_ClearPendingIRQ(SERCOM0_IRQn);

  // Set interrupt priority
  NVIC_SetPriority(SERCOM0_IRQn, 0);
}

/*
  Initialize SERCOM0
*/
void SPISlave::initialize_sercom() {
  /*
    Reset SERCOM0 registers
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
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM0_CORE) | // Select the SERCOM0 core clock
                      GCLK_CLKCTRL_GEN_GCLK0 |            // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;                 // Enable the clock
  
  // Wait for clock register synchronization
  while (GCLK->STATUS.bit.SYNCBUSY);

  /*
    Set up SERCOM0 SPI
  */
  // Set up SPI control A register
  SERCOM0->SPI.CTRLA.reg =  SERCOM_SPI_CTRLA_MODE_SPI_SLAVE | // Set SPI to slave mode
                            SERCOM_SPI_CTRLA_DOPO(0) |        // Set MISO to PAD0, SCK to PAD1, SS to PAD2
                            SERCOM_SPI_CTRLA_DIPO(3);         // Set MOSI to PAD3

  // Enable receiver
  SERCOM0->SPI.CTRLB.bit.RXEN = 1;

  // Wait for CTRLB register synchronization
  while (SERCOM0->SPI.SYNCBUSY.bit.CTRLB);

  // Set up SERCOM0 interrupts
  SERCOM0->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_RXC | // Enable receive complete interrupt
                              SERCOM_SPI_INTENSET_TXC | // Enable transmit complete interrupt
                              SERCOM_SPI_INTENSET_DRE;  // Enable data register empty interrupt

  // Enable SERCOM0
  SERCOM0->SPI.CTRLA.bit.ENABLE = 1;

  // Wait for enable synchronization
  while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE);
}

void SPISlave::init() {
  // Configure SPI pins
  this->configure_spi_pins();

  // Initialize SERCOM interrupt vector
  this->initialize_sercom_irq();

  // Initialize SERCOM
  this->initialize_sercom();

  // Enable SERCOM0 interrupt
  NVIC_EnableIRQ(SERCOM0_IRQn);
}