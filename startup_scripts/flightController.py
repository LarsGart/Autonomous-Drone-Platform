r'''
Author: Jerin Abraham

This is the main flight controller script

CHANNEL MAPPINGS

    CH1 (ROLL): LEFT = 1000, RIGHT = 2000
    CH2 (PITCH): DOWN = 1000, UP = 2000
    CH3 (THROTTLE): DOWN = 1000, UP = 2000
    CH4 (YAW): LEFT = 1000, RIGHT = 2000

MOTOR MAPPINGS
 
    3  ^  0
     \_|_/
     |   |
     |___|
     /   \
    2     1

'''

# Python modules
from time import sleep
from numpy import clip
import serial
import pylibi2c

# Custom modules
from ibus import IBUS
from orientationSensor import OrientationSensor

# Define the UARTs
uart1 = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200
)

uart2 = serial.Serial(
    port="/dev/ttyS0",
    baudrate=115200
)

# Define the i2c busses
accelmag = pylibi2c.I2CDevice('/dev/i2c-0', 0x1F)
gyro = pylibi2c.I2CDevice('/dev/i2c-0', 0x21)

# Instantiate the ibus
ib = IBUS(uart1)

# Instantiate the orientation sensor
sensor = OrientationSensor(accelmag, gyro)

# Split speeds into MSB and LSB for each speed and send byte stream to speed controller via UART2
def outputSpeeds(speeds):
    uart2.write(bytearray([
        60, # Sends a '<'
        (speeds[0] >> 8) & 255, speeds[0] & 255,
        (speeds[1] >> 8) & 255, speeds[1] & 255,
        (speeds[2] >> 8) & 255, speeds[2] & 255,
        (speeds[3] >> 8) & 255, speeds[3] & 255,
        62 # Sends a '>'
    ]))

def main():
    while 1:
        # Read RX values
        rxData = ib.readIBUS()

        if (rxData):
            # Convert inputs to [-1, 1] range ([0, 1] for throttle) range for usability
            rxDataConv = [
                (rxData[0] - 1500) / 500,
                (rxData[1] - 1500) / 500,
                (rxData[2] - 1000) / 1000,
                (rxData[3] - 1500) / 500 
            ]
            
            # Generate motor speeds
            rawSpeeds = [
                0.2 * rxDataConv[2] + 0.1 * rxDataConv[0] - 0.1 * rxDataConv[1] + 0.1 * rxDataConv[3],
                0.2 * rxDataConv[2] + 0.1 * rxDataConv[0] + 0.1 * rxDataConv[1] - 0.1 * rxDataConv[3],
                0.2 * rxDataConv[2] - 0.1 * rxDataConv[0] + 0.1 * rxDataConv[1] + 0.1 * rxDataConv[3],
                0.2 * rxDataConv[2] - 0.1 * rxDataConv[0] - 0.1 * rxDataConv[1] - 0.1 * rxDataConv[3]
            ]

            # Convert speeds to [1000, 2000] range for motor ouputs
            outSpeeds = clip([1000 * i + 1000 for i in rawSpeeds], 1000, 2000)
            outputSpeeds(list(map(int, outSpeeds)))

if __name__ == '__main__':
    sleep(1)
    try:
        main()
    except KeyboardInterrupt:
        # Kill motors
        outputSpeeds([1000] * 4)

        # Close i2c busses
        accelmag.close()
        gyro.close()