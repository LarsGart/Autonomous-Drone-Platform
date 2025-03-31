import spidev
# import Jetson.GPIO as GPIO

SPI_BUS = 0          # spidev0
SPI_SS  = 0          # spidev0.0
SPI_CLOCK = 100000  # 1 Mhz
DATA_TO_SEND = [0x44]

# setup SPI
spi = spidev.SpiDev(SPI_BUS, SPI_SS)
spi.max_speed_hz = SPI_CLOCK
spi.mode = 0

# setup GPIO
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(24, GPIO.OUT, initial=GPIO.HIGH)

def sendData(cmd, bytes):
   spi.xfer([cmd] + bytes)

def readData(cmd, num_bytes):
   values = spi.xfer([cmd] + (num_bytes * [0xFF]))
   # GPIO.output(24, GPIO.HIGH) # In case xfer doesn't release CS
   return values[1:]

try:
   # for i in range(2):
      # sendData(0x45, [0x42, 0x43])
      # values = readData(0x41, 2)
      values = spi.xfer([0x45, 0x44, 0x43])
      values2 = spi.xfer([0xaa, 0xbb, 0xcc])
      print(values, values2)
finally:
   # close SPI
   spi.close()
   # close GPIO
   # GPIO.cleanup()