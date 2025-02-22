import spidev

SPI_BUS = 0          # spidev0
SPI_SS  = 0          # spidev0.0
SPI_CLOCK = 1000000  # 1 Mhz
DATA_TO_SEND = [0xEF, 0xAB, 0xCD, 0x12]

# setup SPI
spi = spidev.SpiDev(SPI_BUS, SPI_SS)
spi.max_speed_hz = SPI_CLOCK

try:
   # transmit byte
   spi.writebytes(DATA_TO_SEND)
finally:
   # close SPI
   spi.close()