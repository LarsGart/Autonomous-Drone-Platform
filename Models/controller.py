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
import logging
from datetime import datetime
import re

sys.path.append("/home/drone/Autonomous-Drone-Platform/models")

from motor import Motors
from rx import RX as Receiver
from pid import PID
from pid import resetError
from zed import Zed


SOCKET = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
SOCKET.setblocking(False)

RX = Receiver()
MOTORS = Motors()
ZED = Zed()

SAMPLE_RATE = 50
PID_LIMIT = 3
THROTTLE_CUTOFF = 1012
THROTTLE_SCALE = 0.5

KP = [0.05, 0.00, 0.05]
KI = [0.00, 0.00, 0.00]
KD = [0.00, 0.00, 0.00]

ROLL  = PID(KP[0], KI[0], KD[0], PID_LIMIT)
PITCH = PID(KP[1], KI[1], KD[1], PID_LIMIT)
YAW   = PID(KP[2], KI[2], KD[2], PID_LIMIT)

class Controller():
   def __init__(self):
      logging.basicConfig(filename=f"/home/drone/Autonomous-Drone-Platform/Logs/{self.__class__.__name__}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
                           ,level=logging.DEBUG
                           ,format='%(asctime)s:%(levelname)s:%(message)s')
      self.logger = logging.getLogger()
      self.logger.info(f"\nP = {KP}\nI = {KI}\nD = {KD}")
      self.logger.info("Controller initialized.") # TODO: Controller should inherit from a base logger class
   
   def write_speeds(self, speeds: list):
      reordered_speeds = [
         speeds[0],
         speeds[3],
         speeds[1],
         speeds[2]
      ]
      MOTORS.write_to_uart(reordered_speeds)

   def run(self):
      self.logger.info('Entering controller loop.')
      pSet = ZED.get_pos_global()
      p0 = pSet
      p1 = None
      dp = None

      while True:
         rx_data = RX.readRX()
         angles = ZED.get_euler_in_degrees()
         
         throttle_norm = (rx_data[2] - 1000) / 1000
         throttle_scaled = 100 * THROTTLE_SCALE * throttle_norm

         roll_set = 10 * ((rx_data[0] - 1500) / 500)
         pitch_set = 10 * ((rx_data[1] - 1500) / 500)
         yaw_set = 10 * ((rx_data[3] - 1500) / 500)

         roll_comp = ROLL.calc(roll_set, -angles['roll'])
         pitch_comp = PITCH.calc(pitch_set, -angles['pitch'])
         yaw_comp = YAW.calc(yaw_set, ZED.get_y_angular_velocity())

         motor_speeds = [
            throttle_scaled + pitch_comp + roll_comp + yaw_comp,
            throttle_scaled - pitch_comp + roll_comp - yaw_comp,
            throttle_scaled - pitch_comp - roll_comp + yaw_comp,
            throttle_scaled + pitch_comp - roll_comp - yaw_comp
         ]

         if (rx_data[2] > THROTTLE_CUTOFF):
            self.write_speeds(motor_speeds)
         else:
            MOTORS.zero_throttle()
            resetError()

         # Calculate drift
         # lateral_error = ZED.get_pos_relative()
         # x_comp = 1000 * self.pid_roll.calc(0, lateral_error[0])
         # z_comp = 1000 * self.pid_yaw.calc(0, lateral_error[2])
         # # self.logger.info(f"dX: {x_comp}\tdZ: {z_comp}")

         # # Normalize throttle
         # throttle_norm = (rx_data[2] - 1000) / 1000

         # # Scale throttle
         # throttle_scaled = 100 * THROTTLE_SCALE * throttle_norm

         # motor_speeds = [
         #    np.clip(int(throttle_scaled + x_comp + z_comp), 0, 100),
         #    np.clip(int(throttle_scaled + x_comp - z_comp), 0, 100),
         #    np.clip(int(throttle_scaled - x_comp + z_comp), 0, 100),
         #    np.clip(int(throttle_scaled - x_comp - z_comp), 0, 100)
         # ]

         # if (rx_data[2] > THROTTLE_CUTOFF):
         #    MOTORS.write_speeds(motor_speeds)
         # else:
         #    MOTORS.zero_throttle()
         # pDelta = ZED.get_position_diff()
         # angles = ZED.get_euler()

         # pid_setpoints = [
         #     0.12 * rx_data[3] - 180,
         #     -0.03 * rx_data[1] + 45,
         #     -0.03 * rx_data[0] + 45
         # ]

         # out_speeds = [int(THROTTLE_SCALE * rx_data[2]) + 500] * 4
         # MOTORS.write_speeds(out_speeds)
         # sleep(1 / SAMPLE_RATE)

   def close(self):
      MOTORS.zero_throttle()
      SOCKET.close()
      ZED.close()
      print(f"\nP = {KP}\nI = {KI}\nD = {KD}")

