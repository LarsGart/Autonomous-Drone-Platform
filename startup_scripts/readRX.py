import time
import serial
import ibus

# Define the UART
uart = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)
# Wait a second to let the port initialize
time.sleep(1)

'''
Function: storeRxData

Desc: reads incoming serial data from RX and stores it in RAM to be
accessed by other scripts

Args: rxDataIn - list of channel data from the RX in microseconds

returns: None
'''
def storeRxData(rxDataIn):
    # TODO: add stuff to write rx data to memory
    pass

# Start IBUS reading loop
ib = ibus.IBUS(
    uart=uart,
    sensor_types=None,
    servo_cb=storeRxData,
    do_log=False
)
ib.start_loop()