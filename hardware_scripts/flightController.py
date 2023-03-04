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
import sys
import numpy as np

from time import sleep, time # I'm sleepy

sys.path.append("/home/drone/Autonomous-Drone-Platform/ZED_Integration")

# Custom modules
from motor_model import Motors
from rx_model import RX
from pid_model import PID
from zed_model import ZedModel

################ DRONE VARS #################

# Throttle scaling
throttleScale = 0.5

#################### PID ####################

# PID coefficients [Yaw, Pitch, Roll]
kP = [1, 1.5, 1.5]
kI = [0, 0.05, 0.05]
kD = [0, 25, 25]

# Define how much of an effect the PID has on motor speeds
pidLimit = 200

#############################################

# Instantiate stuff
motors = Motors()
rx = RX()
zed = ZedModel()

# Create PID
pidPitch = PID(kP[0], kI[0], kD[0], pidLimit)
pidRoll = PID(kP[1], kI[1], kD[1], pidLimit)
pidYaw = PID(kP[2], kI[2], kD[2], pidLimit)

# Instantiate socket
sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
sock.setblocking(False)
sock.bind(('0.0.0.0', 44444))

# Update PID values with values received from the socket and reset the errors
# def readSocketPID():
#     try:
#         data, _ = sock.recvfrom(64)
#         msg = data.decode().strip()
#         if (msg == 'r'):
#             resetErr()
#         else:
#             if re.findall('^[pid][123]=[0-9]+[.][0-9]+$', msg):
#                 arr = (msg[0] == 'p' and kP or msg[0] == 'i' and kI or kD)
#                 arr[int(msg[1]) - 1] = float(msg[3:])
#                 resetErr()

#     except BlockingIOError:
#         pass

def main():
    while 1:
        # Get receiver input
        rxData = RX.readRX()

        # Get PID values from socket if there's an update
        # readSocketPID()

        # Get drone orientation (gay)
        angs = zed.get_euler()

        # Create pid set points
        pidSetPoints = [
            0.12 * rxData[3] - 180,
            -0.03 * rxData[1] + 45,
            -0.03 * rxData[0] + 45
        ]

        # Calculate PID


        # # Calculate pid errors
        # calcErr(droneAngs)

        # # Calculate PID outputs
        # outSpeeds = calcPID(rxData[2])

        outSpeeds = [int(throttleScale * rxData[2]) + 500] * 4

        # Output motor speeds
        motors.outputSpeeds(outSpeeds)

        sleep(1 / sampleRate)

if __name__ == '__main__':
    sleep(1)
    try:
        main()
    except KeyboardInterrupt:
        print("Received termination command")

        # Kill motors
        motors.outputSpeeds([1000] * 4)

        # Close socket
        sock.close()

        # Close zed camera
        zed.closeCamera()
        del zed

        # Print PID values
        print(f"\nP = {kP}\nI = {kI}\nD = {kD}")
