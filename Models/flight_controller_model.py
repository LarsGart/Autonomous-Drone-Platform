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
import time
import logging
from datetime import datetime
import re

sys.path.append("/home/drone/Autonomous-Drone-Platform/Models")

from motor_model import Motors
from rx_model import RX
from pid_model import PID
from zed_model import ZedModel


class FlightController:

   def __init__(self):
      logging.basicConfig(filename=f"../Logs/{self.__class__.__name__}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
                           ,level=logging.DEBUG
                           ,format='%(asctime)s:%(levelname)s:%(message)s')
      self.logger = logging.getLogger()
   
      self.throttle_scale = 0.5
      self.pid_limit = 0.02
      self.kP = [0.15, 0, 0.5]
      self.kI = [0, 0, 0]
      self.kD = [0, 0, 0]
      self.sample_rate = 50

      self.motors = Motors()
      self.motor_tests_pass = self.motors.test_motors()

      if not self.motor_tests_pass:
         self.logger.error("Motor Tests Failed")

      self.rx = RX()
      self.zed = ZedModel()
      self.pid_X = PID(self.kP[0], self.kI[0], self.kD[0], self.pid_limit)
      self.pid_Y = PID(self.kP[1], self.kI[1], self.kD[1], self.pid_limit)
      self.pid_Z = PID(self.kP[2], self.kI[2], self.kD[2], self.pid_limit)

      self.sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
      self.sock.setblocking(False)
      try: # closes any existing socket. if not, initializes connection.
            self.sock.close()
      except:
            self.sock.bind(('0.0.0.0', 44444))

      self.logger.info("FlightController initialized")
   

   # Update PID values with values received from the socket and reset the errors
   def readSocketPID(self):
      try:
         data, _ = self.sock.recvfrom(64)
         msg = data.decode().strip()
         self.logger.info(f"Received: {msg}")
         if (msg == 'r'):
            PID.resetError()
         else:
            if re.findall('^[pid][123]=[0-9]+[.][0-9]+$', msg):
               arr = (msg[0] == 'p' and self.kP or msg[0] == 'i' and self.kI or self.kD)
               arr[int(msg[1]) - 1] = float(msg[3:])
               PID.resetError()

      except BlockingIOError:
         self.logger.error("Socket BlockingIOError")
         pass

      except OSError:
         self.logger.error("Socket OSError")
         pass


   def run(self):
      while True:
         # Get RX data
         rx_data = self.rx.readRX()

         # Read PID values from socket
         # self.readSocketPID()

         # Calculate drift
         lateral_error = self.zed.get_pos_relative()
         x_comp = 10000 * self.pid_X.calc(0, lateral_error[0])
         z_comp = 10000 * self.pid_Z.calc(0, lateral_error[2])
         # self.logger.info(f"dX: {x_comp}\tdZ: {z_comp}")

         # Normalize throttle
         throttle_norm = (rx_data[2] - 1000) / 1000

         # Scale throttle
         throttle_scaled = 1000 * self.throttle_scale * throttle_norm + 1000

         mSpeeds = [
            np.clip(int(throttle_scaled + x_comp + z_comp), 1000, 2000),
            np.clip(int(throttle_scaled + x_comp - z_comp), 1000, 2000),
            np.clip(int(throttle_scaled - x_comp + z_comp), 1000, 2000),
            np.clip(int(throttle_scaled - x_comp - z_comp), 1000, 2000)
         ]

         if (rx_data[2] > 1012):
            self.motors.outputSpeeds(mSpeeds)
         else:
            self.motors.outputSpeeds([1000] * 4)
         # pDelta = self.zed.get_position_diff()
         # angs = self.zed.get_euler()

         # pid_setpoints = [
         #     0.12 * rx_data[3] - 180,
         #     -0.03 * rx_data[1] + 45,
         #     -0.03 * rx_data[0] + 45
         # ]

         # out_speeds = [int(self.throttle_scale * rx_data[2]) + 500] * 4
         # self.motors.outputSpeeds(out_speeds)
         # sleep(1 / self.sample_rate)


   def close(self):
      self.motors.outputSpeeds([1000] * 4)
      self.sock.close()
      self.zed.closeCamera()
      print(f"\nP = {self.kP}\nI = {self.kI}\nD = {self.kD}")
      