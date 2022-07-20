r'''
Author: Jerin Abraham

This is the main flight controller script

CHANNEL MAPPINGS

    CH1 (ROLL): LEFT = 1000, RIGHT = 2000
    CH2 (PITCH): DOWN = 1000, UP = 2000
    CH3 (THROTTLE): DOWN = 1000, UP = 2000
    CH4 (YAW): LEFT = 1000, RIGHT = 2000

MOTOR MAPPINGS
 
    3  ^  0
     \_|_/
     |   |
     |___|
     /   \
    2     1

'''

# Python modules
import re
import serial
import socket
import numpy as np

from time import sleep, time

# Custom modules
import imufusion
from ibus import IBus
from orientationSensor import OrientationSensor

################ DRONE VARS #################

# Sensor sample rate
sampleRate = 100

# Throttle scaling
throttleScale = 0.5

# Mag calibration info generated from spherical fitting
# TODO: Implement onboard magnetometer calibration
r, mx0, my0, mz0 = 50.105256281553686, -68.08280561, -86.10432325, 65.38094172

#################### PID ####################

pidSetPoints = [0, 0, 0]

# PID errors
err = [0, 0, 0]
deltaErr = [0, 0, 0]
errSum = [0, 0, 0]
prevErr = [0, 0, 0]

# PID coefficients [Yaw, Pitch, Roll]
kP = [1, 1, 1]
kI = [0, 0, 0]
kD = [0, 0, 0]

# Define how much of an effect the PID has on motor speeds
pidLimit = 200

#############################################

# Define the UARTs
uart1 = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200
)

uart2 = serial.Serial(
    port="/dev/ttyS0",
    baudrate=115200
)

# Instantiate the ibus
ib = IBus(uart1)

# Instantiate the orientation sensor
sensor = OrientationSensor('/dev/i2c-0')

# Instantiate fusion interface
offset = imufusion.Offset(sampleRate)
ahrs = imufusion.Ahrs()

# Define rx info
rxData = [1500, 1500, 1000, 1500]
rxMissedReadings = 0

# Instantiate socket
sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
sock.setblocking(False)
sock.bind(('0.0.0.0', 44444))

# Split speeds into MSB and LSB for each speed and send byte stream to speed controller via UART2
def outputSpeeds(speeds):
    uart2.write(bytearray([
        60, # Sends a '<'
        (speeds[0] >> 8) & 255, speeds[0] & 255,
        (speeds[1] >> 8) & 255, speeds[1] & 255,
        (speeds[2] >> 8) & 255, speeds[2] & 255,
        (speeds[3] >> 8) & 255, speeds[3] & 255,
        62 # Sends a '>'
    ]))

# Create a 16us deadzone around the center of the joystick to prevent pid errors
def filterRxIn(rxInput):
    return ((rxInput > 1492 and rxInput < 1508) and 1500 or rxInput)

# Calculate pid errors
def calcErr(droneAngs):
    for i in range(3):
        # Calculate P error
        err[i] = droneAngs[i] - pidSetPoints[i]

        # Calculate I error if I coefficient is > 0
        if (kI[i] > 0):
            rawErrSum = errSum[i] + err[i]

            errSum[i] = np.clip(rawErrSum, -pidLimit / kI[i], pidLimit / kI[i])
        
        # Calculate D error
        deltaErr[i] = err[i] - prevErr[i]

        # Update previous error
        prevErr[i] = err[i]

# Calculate motor speeds using PID
def calcPID(throttle):
    pid = [0] * 3
    mOut = [1000] * 4

    # Scale throttle to prevent drone from going into orbit
    throttleMap = throttleScale * throttle + 1000 * (1 - throttleScale)

    # Calculate PID if there's throttle
    if (throttle > 1012):
        for i in range(3):
            pid[i] = (kP[i] * err[i]) + (kI[i] * errSum[i]) + (kD[i] * deltaErr[i])
        
    # Constrain PID values
    pid = np.clip(pid, -pidLimit, pidLimit)

    # Calculate motor speeds
    mOut = [
        int(throttleMap + pid[0] - pid[1] - pid[2]),
        int(throttleMap - pid[0] + pid[1] - pid[2]),
        int(throttleMap + pid[0] + pid[1] + pid[2]),
        int(throttleMap - pid[0] - pid[1] + pid[2])
    ]

    return mOut

# Update PID values with values received from the socket and reset the errors
def readSocketPID():
    try:
        data, _ = sock.recvfrom(64)
        msg = data.decode().strip()
        msgre = re.findall('^[pid][123]=[0-9]+[.][0-9]+$', msg)

        if msgre:
            arr = (msg[0] == 'p' and kP or msg[0] == 'i' and kI or kD)
            arr[int(msg[1]) - 1] = float(msg[3:])

            # Reset error values
            for i in range(3):
                err[i] = 0
                deltaErr[i] = 0
                errSum[i] = 0
                prevErr[i] = 0

    except BlockingIOError:
        pass

# Calculate true drone orientation
def getOrientation(tDelta):
    # Read sensor
    acc, mag, gyr = sensor.readSensor()

    # Apply calibration to magnetometer and gyroscope
    mag = (mag - np.array([mx0, my0, mz0]).transpose()) / r
    gyr = offset.update(gyr)

    # Update fusion filter
    ahrs.update(gyr, acc, mag, tDelta)

    # Get yaw, pitch, and roll info
    eul = ahrs.quaternion.to_euler()

    # Calculate true yaw, pitch, roll
    droneAngs = [
        gyr[2],
        (eul[0] < 0 and 180 + eul[0] or eul[0] - 180),
        -eul[1]
    ]

    return droneAngs

# Read receiver and handle missed inputs
def getControlInputs():
    global rxMissedReadings
    global rxData

    # Flush serial buffer
    uart1.reset_input_buffer()

    # Read RX values
    rxIn = ib.readIBUS()

    # Only update rxData if rxIn isn't None and increment rxMissedReadings if it is
    if (rxIn):
        rxData = np.clip(rxIn, 1000, 2000)
        rxMissedReadings = 0
    else:
        rxMissedReadings += 1

    # If there were over 100 missed readings, set the control input to hover
    if (rxMissedReadings > 100):
        rxData = [1500, 1500, 1000, 1500]

    # Convert inputs to [-1, 1] range ([0, 1] for throttle) range for usability
    rxDataConv = [
        (rxData[0] - 1500) / 500,
        (rxData[1] - 1500) / 500,
        (rxData[2] - 1000) / 1000,
        (rxData[3] - 1500) / 500 
    ]

def main():
    # Initialize filter
    ahrs.settings = imufusion.Settings(
        0.5,  # gain
        10,  # acceleration rejection
        20,  # magnetic rejection
        5 * sampleRate  # rejection timeout = 5 seconds
    )

    tPrev = time()
    while 1:
        # Get time between sensor readings
        tCurr = time()
        tDelta = tCurr - tPrev
        tPrev = tCurr

        # Get receiver input
        getControlInputs()

        # Get PID values from socket if there's an update
        readSocketPID()

        # Get drone orientation
        droneAngs = getOrientation(tDelta)

        # Create pid set points
        pidSetPoints[0] = 0.12 * filterRxIn(rxData[3]) - 180
        pidSetPoints[1] = -0.06 * filterRxIn(rxData[1]) + 90
        pidSetPoints[2] = -0.06 * filterRxIn(rxData[0]) + 90

        # Calculate pid errors
        calcErr(droneAngs)

        # Calculate PID outputs
        outSpeeds = calcPID(rxData[2])

        # Output motor speeds
        outputSpeeds(outSpeeds)

        sleep(1 / sampleRate)

if __name__ == '__main__':
    sleep(1)
    try:
        main()
    except KeyboardInterrupt:
        # Kill motors
        outputSpeeds([1000] * 4)

        # Close sensor
        del sensor

        # Print PID values
        print(f"P = {kP}\nI = {kI}\nD = {kD}")