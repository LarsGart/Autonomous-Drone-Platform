r'''
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

import sys
import socket
import numpy as np
from time import sleep # I'm sleepy

sys.path.append("/home/drone/Autonomous-Drone-Platform/Models")

from motor_model import Motors
from rx_model import RX
from pid_model import PID
from zed_sensor_model import ZedSensorModel


class FlightController:

    def __init__(self):
        self.throttle_scale = 0.5
        self.pid_limit = 200
        self.kP = [1, 1.5, 1.5]
        self.kI = [0, 0.05, 0.05]
        self.kD = [0, 25, 25]
        self.sample_rate = 50

        self.motors = Motors()
        self.rx = RX()
        self.zed = ZedSensorModel()
        self.pid_pitch = PID(self.kP[0], self.kI[0], self.kD[0], self.pid_limit)
        self.pid_roll = PID(self.kP[1], self.kI[1], self.kD[1], self.pid_limit)
        self.pid_yaw = PID(self.kP[2], self.kI[2], self.kD[2], self.pid_limit)

        self.sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(('0.0.0.0', 44444))

    def run(self):
        while True:
            rx_data = self.rx.readRX()
            angs = self.zed.get_euler()
            depth_array = self.zed.get_depth_array()
            goal = (depth_array.shape[0]//2, depth_array.shape[1]//2)

            velocity_commands = self.obstacle_avoidance_policy(depth_array, goal)

            pid_setpoints = [
                0.12 * rx_data[3] - 180 + velocity_commands[1],
                -0.03 * rx_data[1] + 45 + velocity_commands[0],
                -0.03 * rx_data[0] + 45 + velocity_commands[0]
            ]

            out_speeds = [int(self.throttle_scale * rx_data[2]) + 500] * 4
            self.motors.outputSpeeds(out_speeds)
            sleep(1 / self.sample_rate)

    def close(self):
        self.motors.outputSpeeds([1000] * 4)
        self.sock.close()
        self.zed.closeCamera()
        print(f"\nP = {self.kP}\nI = {self.kI}\nD = {self.kD}")

if __name__ == '__main__':
    sleep(1)
    flight_controller = FlightController()
    try:
        flight_controller.run()
    except KeyboardInterrupt:
        print("Received termination command")
        flight_controller.close() 
