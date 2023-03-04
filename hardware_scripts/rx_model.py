'''
Author: Jerin Abraham
'''
import serial
import numpy as np
from ibus_model import IBus

'''
Class to instantiate an RX object
'''
class RX():
    '''
    Constructor for the RX class
    Instantiates the UART and IBUS and initializes the outputs
    '''
    def __init__(self):
        # Define the UART
        self.uart = serial.Serial(
            port="/dev/ttyTHS1",
            baudrate=115200
        )

        # Instantiate the ibus
        self.ibus = IBus(self.uart)

        # Define RX defaults
        self.input = [0] * 14
        self.output = [1500] * 4
        self.missedReadings = 0

    '''
    Destructor for the RX class
    Closes and deletes the UART port
    '''
    def __del__(self):
        # Close the UART port
        del self.uart

    '''
    Create a 16us deadzone around the center of the joystick to prevent pid errors

    PARAMETERS:
        input (int): an output value from one of the channels of the receiver

    RETURNS:
        (int): integer value with a deadband
    '''
    def createDeadband(self, input):
        return ((input > 1492 and input < 1508) and 1500 or input)
    
    # Read receiver and handle missed inputs
    def getCtrlInput(self):
        # Flush serial buffer
        self.uart.reset_input_buffer()

        # Read RX values
        self.input = self.ibus.readIBUS()

        # Only update rxData if rxIn isn't None and increment rxMissedReadings if it is
        if (self.input):
            self.output = np.clip(self.input, 1000, 2000)
            self.missedReadings = 0
        else:
            self.missedReadings += 1

        # If there were over 100 missed readings, set the control input to hover
        if (self.missedReadings > 100):
            self.output = [1500] * 4

        return self.output