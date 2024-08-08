'''
Author: house4hack
Editor: jerinabr

Credit: https://github.com/house4hack/circuitpython-ibus
'''

from models.logger import Logger

PROTOCOL_SERVO = 0x40
PROTOCOL_CHANNELS = 14
PROTOCOL_OVERHEAD = 3
PROTOCOL_LENGTH = 0x20

class IBus(Logger):
    def __init__(self, uart):
        self.uart = uart
        super().__init__()

    def readUART(self):
        data = None
        while data is None:
            data = self.uart.read(1)
        return data
    
    def checksum(self, arr, initial):
        sum = initial
        for val in arr:
            sum += val
        checksum = 0xFFFF - sum
        chA = checksum >> 8
        chB = checksum & 0xFF
        return chA, chB

    def readIBUS(self):
        data = self.readUART()
        expectedLen = data[0] - 1
        if PROTOCOL_OVERHEAD <= expectedLen < PROTOCOL_LENGTH:
            dataArr = bytearray(expectedLen)
            totalRead = self.uart.readinto(dataArr)
            if totalRead == expectedLen:
                cmd = dataArr[0] & 0xF0
                chA1, chB1 = dataArr[-1], dataArr[-2]
                chA2, chB2 = self.checksum(dataArr[:-2], expectedLen + 1)
                if chA1 == chA2 and chB1 == chB2:
                    if cmd == PROTOCOL_SERVO:
                        return [(dataArr[2 * i + 2] << 8) | dataArr[2 * i + 1] for i in range(PROTOCOL_CHANNELS)]
        return None

