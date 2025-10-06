#include "Jetson_SPI.h"

#define RING_BUFFER_SIZE  32
#define READ_IDLE_BYTES    4
#define SPI_RX_DMA         0
#define SPI_TX_DMA         1

// Initialize the Jetson SPI instance
Jetson_SPI_t jetson_spi = {
  .received_cmd = 0,
  .rd_data = {0},
  .rd_data_ready = false
};

// 2 DMA descriptors: one for RX, one for TX
DmacDescriptor dma_desc[2] __attribute__((aligned(16)));
DmacDescriptor dma_wrb[2] __attribute__((aligned(16)));

// Constants
static const uint16_t SPI_START_BYTES = 0xBC9E; // Start bytes for SPI communication
static const uint8_t SPI_DATA_LENGTH[NUM_CMDS] = {
  1, // ESC_ARM_DISARM
  8, // MOTOR_SPEEDS
  1, // MOTOR_STOP
  1  // READ_REGISTER
};

// Variables
static uint8_t rd_buffer[RING_BUFFER_SIZE]; // Buffer to hold received data
static uint8_t rd_buffer_rd_ptr = 0;
static uint8_t wr_buffer[RING_BUFFER_SIZE]; // Buffer to hold data to write
static uint8_t wr_buffer_rd_ptr = 0;

// ----------------------------------------------------------------
// CRC-16-CCITT TABLE
// ----------------------------------------------------------------
static const uint16_t crc16_table[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};
// ----------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------
/*!
  \brief Compute the CRC-16-CCITT checksum for a byte using a lookup table and updates the crc pointer
  \param crc The pointer to the CRC value that is being updated
  \param data The byte that is being used to calculate the CRC
*/
static void compute_crc16(uint16_t *crc, uint8_t data) {
  *crc = (*crc << 8) ^ crc16_table[((*crc >> 8) ^ data) & 0xFF];
}

/*!
  \brief Check if there is data available in the SPI receive buffer
  \return true if data is available, false otherwise
*/
static inline bool isDataAvailable(void) {
  uint16_t wr_ptr = RING_BUFFER_SIZE - dma_wrb[SPI_RX_DMA].BTCNT.reg;
  return rd_buffer_rd_ptr != wr_ptr;
}

/*!
  \brief Read a byte from the SPI receive buffer
  \return The next byte from the buffer, or 0 if the buffer is empty
*/
static inline uint8_t readByte(void) {
  uint8_t data = rd_buffer[rd_buffer_rd_ptr++];
  if (rd_buffer_rd_ptr >= RING_BUFFER_SIZE) {
    rd_buffer_rd_ptr = 0; // Wrap around if needed
  }
  return data;
}

/*!
  \brief Process a received byte from the SPI master and manage the state machine for packet reception
  \param data The received byte
*/
static inline void process_received_byte(uint8_t data) {
  static enum {
    WAIT_FOR_START_1,
    WAIT_FOR_START_2,
    WAIT_FOR_CMD,
    WAIT_FOR_DATA,
    WAIT_FOR_CRC_1,
    WAIT_FOR_CRC_2
  } state = WAIT_FOR_START_1;

  static uint8_t expected_data_size = 0;
  static uint8_t data_index = 0;
  static uint16_t computed_crc = 0xFFFF;
  static uint16_t received_crc = 0;

  switch (state) {
    case WAIT_FOR_START_1: {
      if (data == (SPI_START_BYTES >> 8)) { // Check first start byte
        computed_crc = 0xFFFF; // Reset CRC
        compute_crc16(&computed_crc, data);
        state = WAIT_FOR_START_2;
      }
      break;
    }

    case WAIT_FOR_START_2: {
      if (data == (SPI_START_BYTES & 0xFF)) { // Check second start byte
        compute_crc16(&computed_crc, data);
        state = WAIT_FOR_CMD;
      } else {
        state = WAIT_FOR_START_1; // Reset if not matched
      }
      break;
    }

    case WAIT_FOR_CMD: {
      jetson_spi.received_cmd = data;
      // Validate command and get expected data size
      if (jetson_spi.received_cmd < NUM_CMDS) {
        compute_crc16(&computed_crc, data);
        expected_data_size = SPI_DATA_LENGTH[jetson_spi.received_cmd];
        data_index = 0;
        state = WAIT_FOR_DATA;
      } else {
        state = WAIT_FOR_START_1; // Invalid command, reset
      }
      break;
    }

    case WAIT_FOR_DATA: {
      jetson_spi.rd_data[data_index++] = data;
      compute_crc16(&computed_crc, data);
      if (data_index >= expected_data_size) {
        data_index = 0;
        state = WAIT_FOR_CRC_1;
      }
      break;
    }

    case WAIT_FOR_CRC_1: {
      received_crc = data << 8; // Store first CRC byte
      state = WAIT_FOR_CRC_2;
      break;
    }

    case WAIT_FOR_CRC_2: {
      received_crc |= data; // Store second CRC byte
      if (computed_crc == received_crc) {
        /*
          Track where the read pointer to the write buffer is after a command is processed
          This is done because when the Jetson wants to read, it'll send a read command, then
          4 idle bytes to allow the XIAO enough time to process and load data into the write buffer
          This read pointer is used as an offset for the write pointer when the write buffer is filled
        */
        wr_buffer_rd_ptr = RING_BUFFER_SIZE - dma_wrb[SPI_TX_DMA].BTCNT.reg;
        jetson_spi.rd_data_ready = true;
      }
      state = WAIT_FOR_START_1; // Reset state machine for next packet
      break;
    }

    default: {
      state = WAIT_FOR_START_1; // Reset on unknown state
      break;
    }
  }
}

/*!
  \brief Configure the pins for SERCOM0 SPI slave operation
  
  This function sets the peripheral multiplexer for the SPI pins and sets
  the appropriate pin functions for MISO, MOSI, SCK, and CS
*/
static void configure_pins(void) {
  // Enable pin mux
  PORT->Group[PORTA].PINCFG[4].bit.PMUXEN = 1;
  PORT->Group[PORTA].PINCFG[5].bit.PMUXEN = 1;
  PORT->Group[PORTA].PINCFG[6].bit.PMUXEN = 1;
  PORT->Group[PORTA].PINCFG[7].bit.PMUXEN = 1;

  // Select pin mux function
  PORT->Group[PORTA].PMUX[2].bit.PMUXE = MUX_PA04D_SERCOM0_PAD0;
  PORT->Group[PORTA].PMUX[2].bit.PMUXO = MUX_PA05D_SERCOM0_PAD1;
  PORT->Group[PORTA].PMUX[3].bit.PMUXE = MUX_PA06D_SERCOM0_PAD2;
  PORT->Group[PORTA].PMUX[3].bit.PMUXO = MUX_PA07D_SERCOM0_PAD3;
}

/*!
  \brief Configure SERCOM0 as a SPI slave
*/
static void configure_sercom(void) {
  // Enable SERCOM0 clocks
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;                // Enable APBC clock for SERCOM0
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM0_CORE) | // Select the SERCOM0 core clock
                      GCLK_CLKCTRL_GEN_GCLK0 |            // Use generic clock generator 0
                      GCLK_CLKCTRL_CLKEN;                 // Enable the clock
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Reset SERCOM0 and wait for synchronization
  SERCOM0->SPI.CTRLA.bit.SWRST = 1;
  while (SERCOM0->SPI.CTRLA.bit.SWRST || SERCOM0->SPI.SYNCBUSY.bit.SWRST);

  // Set up SERCOM0 SPI registers
  SERCOM0->SPI.CTRLA.reg =  SERCOM_SPI_CTRLA_MODE_SPI_SLAVE | // Set SPI to slave mode
                            SERCOM_SPI_CTRLA_DOPO(0) |        // Set MISO to PAD0, SCK to PAD1, SS to PAD2
                            SERCOM_SPI_CTRLA_DIPO(3);         // Set MOSI to PAD3
  SERCOM0->SPI.CTRLB.reg =  SERCOM_SPI_CTRLB_RXEN;            // Enable receiver
  while (SERCOM0->SPI.SYNCBUSY.bit.CTRLB);

  // Enable SERCOM0 SPI
  SERCOM0->SPI.CTRLA.bit.ENABLE = 1;
  while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE);
}

/*!
  \brief Configure the DMAC to transfer data between the SPI data register and the read/write circular buffers
*/
static void configure_dmac(void) {
  // Enable clock for DMAC
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC;   // Enable AHB clock for DMAC
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC; // Enable APBB clock for DMAC

  // Reset DMAC
  DMAC->CTRL.bit.DMAENABLE = 0;
  DMAC->CTRL.bit.CRCENABLE = 0;
  DMAC->CTRL.bit.SWRST = 1;
  
  // Configure DMAC descriptors and control
  DMAC->BASEADDR.reg = (uint32_t)&dma_desc; // Set base address for descriptors
  DMAC->WRBADDR.reg = (uint32_t)&dma_wrb; // Set base address for write-back descriptors
  DMAC->CTRL.reg =  DMAC_CTRL_DMAENABLE | // Enable DMAC
                    DMAC_CTRL_LVLEN(0xF); // Enable all priority levels

  /*
    Configure DMA channel 0 for SPI RX
      1. Reset channel
      2. Set up RX descriptor
      3. Configure channel control
      4. Enable channel
  */
  DMAC->CHID.reg = DMAC_CHID_ID(0); // Select channel 0
  DMAC->CHCTRLA.bit.ENABLE = 0;
  DMAC->CHCTRLA.bit.SWRST = 1;
  
  dma_desc[SPI_RX_DMA].BTCTRL.reg = DMAC_BTCTRL_VALID |           // Descriptor is valid
                                    DMAC_BTCTRL_BLOCKACT_NOACT |  // No action on block complete
                                    DMAC_BTCTRL_BEATSIZE_BYTE |   // Transfer size: byte
                                    DMAC_BTCTRL_DSTINC;           // Increment destination address
  dma_desc[SPI_RX_DMA].BTCNT.reg = RING_BUFFER_SIZE;              // Number of bytes to transfer
  dma_desc[SPI_RX_DMA].SRCADDR.reg = (uint32_t)&SERCOM0->SPI.DATA.reg;
  dma_desc[SPI_RX_DMA].DSTADDR.reg = (uint32_t)(rd_buffer + RING_BUFFER_SIZE);
  dma_desc[SPI_RX_DMA].DESCADDR.reg = (uint32_t)&dma_desc[SPI_RX_DMA]; // Circular buffer

  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(1) |                       // Priority level 1
                      DMAC_CHCTRLB_TRIGSRC(SERCOM0_DMAC_ID_RX) |  // Trigger source: SERCOM0 RX
                      DMAC_CHCTRLB_TRIGACT_BEAT;                  // Trigger action: one beat transfer
  DMAC->CHCTRLA.bit.ENABLE = 1;

  /*
    Configure DMA channel 1 for SPI TX
      1. Reset channel
      2. Set up TX descriptor
      3. Configure channel control
      4. Enable channel
  */
  DMAC->CHID.reg = DMAC_CHID_ID(1); // Select channel 1
  DMAC->CHCTRLA.bit.ENABLE = 0;
  DMAC->CHCTRLA.bit.SWRST = 1;

  dma_desc[SPI_TX_DMA].BTCTRL.reg = DMAC_BTCTRL_VALID |           // Descriptor is valid
                                    DMAC_BTCTRL_BLOCKACT_NOACT |  // No action on block complete
                                    DMAC_BTCTRL_BEATSIZE_BYTE |   // Transfer size: byte
                                    DMAC_BTCTRL_SRCINC;           // Increment source address
  dma_desc[SPI_TX_DMA].BTCNT.reg = RING_BUFFER_SIZE;              // Number of bytes to transfer
  dma_desc[SPI_TX_DMA].SRCADDR.reg = (uint32_t)(wr_buffer + RING_BUFFER_SIZE);
  dma_desc[SPI_TX_DMA].DSTADDR.reg = (uint32_t)&SERCOM0->SPI.DATA.reg;
  dma_desc[SPI_TX_DMA].DESCADDR.reg = (uint32_t)&dma_desc[SPI_TX_DMA]; // Circular buffer

  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(0) |                       // Priority level 0
                      DMAC_CHCTRLB_TRIGSRC(SERCOM0_DMAC_ID_TX) |  // Trigger source: SERCOM0 TX
                      DMAC_CHCTRLB_TRIGACT_BEAT;                  // Trigger action: one beat transfer
  DMAC->CHCTRLA.bit.ENABLE = 1;
}

// ----------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------
/*!
  \brief Initialize the Jetson SPI and DMAC
*/
void jetson_spi_init(void) {
  configure_pins();
  configure_sercom();
  configure_dmac();
}

/*!
  \brief Process all available data in the SPI receive buffer
  
  This function should be called periodically in the main loop to handle incoming SPI data
*/
void process_spi_rx_data(void) {
  while (isDataAvailable()) {
    uint8_t data = readByte();
    process_received_byte(data);
  }
}

/*!
  \brief Queue data in the write buffer to be sent to the Jetson.
  \param data Pointer to the data to send
  \param length Number of bytes to send

  This function copies the provided data into the write buffer and updates the write pointer
*/
void write_spi_data(const uint8_t* data, uint8_t length) {
  // Set the write pointer to be 4 bytes ahead of the read pointer to account for the 4 idle bytes
  uint8_t wr_buffer_wr_ptr = wr_buffer_rd_ptr + READ_IDLE_BYTES - 1;
  if (wr_buffer_wr_ptr >= RING_BUFFER_SIZE) {
    wr_buffer_wr_ptr -= RING_BUFFER_SIZE;
  }
  for (uint8_t i = 0; i < length; i++) {
    wr_buffer[wr_buffer_wr_ptr++] = data[i];
    if (wr_buffer_wr_ptr == RING_BUFFER_SIZE) {
      wr_buffer_wr_ptr = 0;
    }
  }
}
