from time import sleep
import serial

from ibus_rewrite import IBUS

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

# Instantiate ibus
ib = IBUS(uart=uart1)

def outputSpeeds(speeds):
    uart2.write(bytearray(
        60,
        speeds[0] >> 8, speeds[0] & 255,
        speeds[1] >> 8, speeds[1] & 255,
        speeds[2] >> 8, speeds[2] & 255,
        speeds[3] >> 8, speeds[3] & 255,
        62
    ))

def main():
    while 1:
        # Read RX values
        rxData = ib.readIBUS()

        if (rxData is not None):
            # Generate motor speeds
            print(rxData)

if __name__ == '__main__':
    sleep(1)
    main()