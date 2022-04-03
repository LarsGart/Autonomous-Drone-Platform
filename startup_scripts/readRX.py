from asyncio.windows_events import NULL
import time
import serial
import ibus

# Define the UART
uart = serial.Serial(
    port = "/dev/ttyTHS1",
    baudrate = 115200,
    bytesize = serial.EIGHTBITS,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE
)
# Wait a second to let the port initialize
time.sleep(1)

def storeRxData(rxDataIn):
    # TODO: add stuff to write rx data to memory
    pass

ib = ibus.IBUS(uart, sensor_types=None, servo_cb=storeRxData, do_log=False)
ib.start_loop()