import time
import serial
from multiprocessing import Process

import ibus

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

# Define ibus and callback
def storeRxData(rxDataIn):
    print(rxDataIn)

ib = ibus.IBUS(
    uart=uart1,
    sensor_types=[ibus.IBUSS_ALT],
    servo_cb=storeRxData,
    do_log=False
)

def main():
    while 1:
        time.sleep(1)
        print("Hello!")

if __name__ == '__main__':
    Process(target=ib.start_loop).start()
    Process(target=main).start()