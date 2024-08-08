'''
Author: Jerin Abraham

This class handles reading from the RC receiver
'''
import serial
import numpy as np
from ibus import IBus

UART = serial.Serial(port="/dev/ttyTHS1", baudrate=115200)
IBUS = IBus(UART)

input = [0] * 14
output = [1500] * 4
missed_readings = 0


class RX:
    '''
    Create a 16us deadzone around the center of the joystick to prevent pid errors

    PARAMETERS:
        input (int): an output value from one of the channels of the receiver

    RETURNS:
        (int): integer value with a deadband
    '''
    def __createDeadband(self, input):
        return input if not 1492 < input < 1508 else 1500

    '''
    Read receiver and handle missed inputs

    RETURNS:
        List[int]: A list of the received values from each channel of the receiver
    '''
    def readRX(self):
        # Flush serial buffer
        UART.reset_input_buffer()

        # Read RX values
        input = IBUS.readIBUS()

        # Only update output if input isn't None and increment missed_readings if it is
        if input:
            output = np.clip(input, 1000, 2000)
            missed_readings = 0
        else:
            missed_readings += 1

            # If there were over 100 missed readings, set the control input to hover
            if missed_readings > 100:
                output = [1500, 1500, 1000, 1500]

        # Create a deadband for channels 0, 1, and 3
        output[0], output[1], output[3] = (self.__createDeadband(input) for input in (output[0], output[1], output[3]))
        return output

    def __del__(self):
        UART.close()

