import serial
from ibus import IBus

uart1 = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200
)

ib = IBus(uart1)

while 1:
    rxIn = ib.readIBUS()
    print(rxIn)