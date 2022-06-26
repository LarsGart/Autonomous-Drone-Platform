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

# Custom modules
from ibus import IBUS

# Define the UARTs
uart1 = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

uart2 = serial.Serial(
    port="/dev/ttyS0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

# Instantiate ibus
ib = IBUS(uart=uart1)

def outputSpeeds(speeds):
    uart2.write(bytearray([
        60, # Sends a '<'
        speeds[0] >> 8, speeds[0] & 255,
        speeds[1] >> 8, speeds[1] & 255,
        speeds[2] >> 8, speeds[2] & 255,
        speeds[3] >> 8, speeds[3] & 255,
        62 # Sends a '>'
    ]))

def main():
    while 1:
        # Read RX values
        rxData = ib.readIBUS()

        if (rxData):
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
            outSpeeds = clip([1000 * i + 1000 for i in rawSpeeds], 1000, 2000)
            outputSpeeds(list(map(int, outSpeeds)))

if __name__ == '__main__':
    sleep(1)
    try:
        main()
    except KeyboardInterrupt:
        outputSpeeds([1000] * 4)