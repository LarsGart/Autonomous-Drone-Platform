'''
Author: house4hack
Editor: jerinabr

Code taken from https://github.com/house4hack/circuitpython-ibus

I removed the parts that weren't needed for this project and did some refactoring
'''

# Protocol constants
PROTOCOL_SERVO = 0x40
PROTOCOL_CHANNELS = 14
PROTOCOL_OVERHEAD = 3
PROTOCOL_LENGTH = 0x20

class IBUS():
    def __init__(self, uart):
        self.uart = uart

    def readUART(self):
        data = self.uart.read(1)
        while data is None:
            data = self.uart.read(1)
        return data

    def checksum(self, arr, initial):
        sum = initial
        for a in arr:
            sum += a
        checksum = 0xFFFF - sum
        chA = checksum >> 8
        chB = checksum & 0xFF
        return chA, chB
    
    def readIBUS(self):
        data = self.readUART()

        expectedLen = data[0] - 1
        if (expectedLen < PROTOCOL_LENGTH and expectedLen >= PROTOCOL_OVERHEAD):
            dataArr = bytearray(expectedLen)
            totalRead = self.uart.readinto(dataArr)
            if (totalRead == expectedLen):
                cmd = dataArr[0] & 0xF0

                chA1 = dataArr[-1]
                chB1 = dataArr[-2]

                chA2, chB2 = self.checksum(dataArr[:-2], expectedLen + 1)
                if (chA1 == chA2 and chB1 == chB2):
                    if (cmd == PROTOCOL_SERVO):
                        return [dataArr[2 * i + 1] | (dataArr[2 * i + 2] << 8) for i in range(PROTOCOL_CHANNELS)]
        
        return None
