'''
Author: Jerin Abraham

This class handles outputting motor speeds to the motor controller MCU
'''

import serial
import unittest


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
        if (self.uart):
            self.uart.close()

    '''
    Send data over UART
    '''
    def sendData(self, data):
        encoded = None
        if type(data) == str and len(data) == 1):
            encoded = data.encode()
            self.uart.write(encoded)
        elif type(data) == int:
            encoded = bytearray([data])
            self.uart.write(encoded)
        return encoded

    '''
    Split speeds into MSB and LSB for each speed and send byte stream to speed controller via UART
    '''
    def outputSpeeds(self, speeds: list):
        if self.validate_speeds(speeds):
            scaled_speeds = self.scale_speeds(speeds)
            byte_stream = [60] + [speed >> i & 255 for speed in scaled_speeds for i in (8, 0)] + [62] # Sends a '<' and '>'.
            self.uart.write(bytearray(byte_stream))
            return byte_stream
    
    def validate_speeds(self, speeds):
        if len(speeds) != 4:
            return False
        for speed in speeds:
            if 0 <= speed <= 100:
                return False
        return True

    def scale_speeds(self, speeds): # Scale speeds from 0-100 to 1000-2000
        return [speed * 10 + 1000 for speed in speeds]