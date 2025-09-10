'''
Author: house4hack
Editor: jerinabr

Code taken from https://github.com/house4hack/circuitpython-ibus
'''

class IBus:
    def __init__(self, uart):
        self.uart = uart
        self.PROTOCOL_SERVO = 0x40
        self.PROTOCOL_CHANNELS = 14
        self.PROTOCOL_OVERHEAD = 3
        self.PROTOCOL_LENGTH = 0x20


    def readUART(self):
        data = None
        while data is None:
            data = self.uart.read(1)
        return data


    #@staticmethod
    def checksum(self, arr, initial):
        # sum = initial + sum(arr)
        # checksum = 0xFFFF - sum
        # return checksum >> 8, checksum & 0xFF
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
        if self.PROTOCOL_OVERHEAD <= expectedLen < self.PROTOCOL_LENGTH:
            dataArr = bytearray(expectedLen)
            totalRead = self.uart.readinto(dataArr)
            if totalRead == expectedLen:
                cmd = dataArr[0] & 0xF0

                chA1, chB1 = dataArr[-1], dataArr[-2]
                chA2, chB2 = self.checksum(dataArr[:-2], expectedLen + 1)
                if chA1 == chA2 and chB1 == chB2:
                    if cmd == self.PROTOCOL_SERVO:
                        return [(dataArr[2 * i + 2] << 8) | dataArr[2 * i + 1] for i in range(self.PROTOCOL_CHANNELS)]

        return None
    