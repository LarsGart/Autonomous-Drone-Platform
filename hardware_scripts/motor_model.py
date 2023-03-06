'''
Author: Jerin Abraham

This class handles outputting motor speeds
'''

import serial


class Motors:
    '''
    Instantiates the UART
    '''
    def __init__(self):
        self.uart = serial.Serial(port="/dev/ttyS0", baudrate=115200)

    '''
    Closes and deletes the UART
    '''
    def __del__(self):
        self.uart.close()

    '''
    Split speeds into MSB and LSB for each speed and send byte stream to speed controller via UART
    '''
    def outputSpeeds(self, speeds):
        values = [60] + [speed >> i & 255 for speed in speeds for i in (8, 0)] + [62] # Sends a '<' and '>'
        self.uart.write(bytearray(values))
        