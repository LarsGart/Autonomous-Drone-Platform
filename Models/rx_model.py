'''
Author: Jerin Abraham

This class handles reading from the RC receiver
'''
import serial
import numpy as np
from ibus_model import IBus


class RX:
    '''
    Initializes the UART and IBUS and initializes the outputs
    '''
    def __init__(self):
        self.uart = serial.Serial(port="/dev/ttyTHS1", baudrate=115200)
        self.ibus = IBus(self.uart)
        self.input = [0] * 14
        self.output = [1500] * 4
        self.missedReadings = 0

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
        self.uart.reset_input_buffer()

        # Read RX values
        self.input = self.ibus.readIBUS()

        # Only update output if input isn't None and increment missedReadings if it is
        if self.input:
            self.output = np.clip(self.input, 1000, 2000)
            self.missedReadings = 0
        else:
            self.missedReadings += 1

            # If there were over 100 missed readings, set the control input to hover
            if self.missedReadings > 100:
                self.output = [1500, 1500, 1000, 1500]

        # Create a deadband for channels 0, 1, and 3
        self.output[0], self.output[1], self.output[3] = (self.__createDeadband(input) \
                                                          for input in (self.output[0]
                                                                        , self.output[1]
                                                                        , self.output[3]))
        return self.output
    
    def readRX_normalized(self):
        return (self.readRX() - 1000) / 1000

    def __del__(self):
        self.uart.close()
        