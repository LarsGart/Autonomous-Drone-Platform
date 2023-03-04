'''
Author: Jerin Abraham

This class handles outputting motor speeds
'''

import serial

class Motors():
    '''
    Constructor for the Motors class
    Instantiates the UART
    '''
    def __init__(self):
        # Define the UART
        self.uart = serial.Serial(
            port="/dev/ttyS0",
            baudrate=115200
        )

    '''
    Destructor for the Motors class
    Closes and deletes the UART
    '''
    def __del__(self):
        # Close the UART
        del self.uart

    '''
    Split speeds into MSB and LSB for each speed and send byte stream to speed controller via UART
    '''
    def outputSpeeds(self, speeds):
        self.uart.write(bytearray([
            60, # Sends a '<'
            (speeds[0] >> 8) & 255, speeds[0] & 255,
            (speeds[1] >> 8) & 255, speeds[1] & 255,
            (speeds[2] >> 8) & 255, speeds[2] & 255,
            (speeds[3] >> 8) & 255, speeds[3] & 255,
            62 # Sends a '>'
        ]))