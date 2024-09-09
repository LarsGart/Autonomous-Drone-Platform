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
import struct
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
from pid_model import resetError as resetError
from zed_model import ZedModel


class FlightController:

   def __init__(self, test_mode=True):
      logging.basicConfig(filename=f"/home/drone/Autonomous-Drone-Platform/Logs/{self.__class__.__name__}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
                           ,level=logging.DEBUG
                           ,format='%(asctime)s:%(levelname)s:%(message)s')
      self.logger = logging.getLogger()

      self.test_mode = True if test_mode == 'test' else False
      self.logger.info(f'Test Mode: {self.test_mode}')
      self.throttle_scale = 0.5
      self.pid_limit = 10
      # self.kP = [0.243*.5, 0.15, 0]
      # self.kI = [0.243*.5, 0.15, 0]
      # self.kD = [0.0151*.5, 0.010125, 0]
      self.kP = [.15, .25, 1]
      self.kI = [7.5, 7.5, 0]
      self.kD = [.00825, .00825, 0]
      self.logger.info(f"\nP = {self.kP}\nI = {self.kI}\nD = {self.kD}")
      self.sample_rate = 50
      self.initialTime = time.time()

      self.throttle_cutoff = 1012
      
      if not self.test_mode:
         self.logger.info('Initializing Motors')
         self.motors = Motors()
         self.motor_tests_pass = self.motors.test_motors()

         if not self.motor_tests_pass:
            self.logger.error("Motor Tests Failed")
         else:
            self.logger.info("Motor Tests Passed")

         self.logger.info("Initializing RX")
         self.rx = RX()
         self.logger.info("Successfully initialized RX")

      self.logger.info("Initializing ZED")
      self.zed = ZedModel()
      self.logger.info("Successfully initialized ZED")
      
      self.pid_roll = PID(self.kP[0], self.kI[0], self.kD[0], self.pid_limit)
      self.pid_pitch = PID(self.kP[1], self.kI[1], self.kD[1], self.pid_limit)
      self.pid_yaw = PID(self.kP[2], self.kI[2], self.kD[2], self.pid_limit)

      self.sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
      self.sock.setblocking(False)
      try: # closes any existing socket. if not, initializes connection.
            pass#self.sock.close()
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
            resetError()
         else:
            if re.findall('^[pid][123]=[0-9]+[.][0-9]+$', msg):
               arr = (msg[0] == 'p' and self.kP or msg[0] == 'i' and self.kI or self.kD)
               arr[int(msg[1]) - 1] = float(msg[3:])
               resetError()

      except BlockingIOError:
         self.logger.error("Socket BlockingIOError")
         pass

      except OSError:
         self.logger.error("Socket OSError")
         pass

   
   def outputSpeeds(self, speeds: list):
      reordered_speeds = [
         speeds[0],
         speeds[3],
         speeds[1],
         speeds[2]
      ]
      self.motors.output_speeds(reordered_speeds)

   def run(self):
      self.logger.info('Entering main event loop')
      pSet = self.zed.get_pos_global()
      p0 = pSet
      p1 = None
      dp = None
      while True:
         # Get RX data
         rx_data = self.rx.readRX()

         # Get angles in degrees
         angs = self.zed.get_euler_in_degrees()
         # print(angs)

         # Normalize throttle
         throttle_norm = (rx_data[2] - 1000) / 1000

         # Scale throttle
         throttle_scaled = 100 * self.throttle_scale * throttle_norm

         # Get relative position difference
         # p1 = self.zed.get_pos_global()
         # absDelta = p1 - pSet
         # absDeltaXZ = [absDelta[0], absDelta[2]]

         # Create set points
         # desiredXVel = -10 * absDeltaXZ[0]
         # desiredZVel = -10 * absDeltaXZ[1]
         rollSet = 10 * ((rx_data[0] - 1500) / 500)
         pitchSet = 10 * ((rx_data[1] - 1500) / 500)
         yawSet = 10 * ((rx_data[3] - 1500) / 500)

         # self.logger.info(f'{rollSet}, {-angs["roll"]}, {pitchSet}, {-angs["pitch"]}')

         # Get current velocity
         # dp = p1 - p0
         # p0 = p1
         # currentXVel = dp[0]
         # currentZVel = dp[1]

         # # Calculate PID values
         # roll_comp = self.pid_roll.calc(desiredXVel, currentXVel)
         # pitch_comp = self.pid_pitch.calc(desiredZVel, currentZVel)
         # yaw_comp = 0
         roll_comp = self.pid_roll.calc(rollSet, -angs['roll'])
         pitch_comp = self.pid_pitch.calc(pitchSet, -angs['pitch'])
         y_angular_velocity = self.zed.get_y_angular_velocity()
         self.logger.info(f'{y_angular_velocity}')

         yaw_comp = self.pid_yaw.calc(yawSet, y_angular_velocity)
         # self.logger.info(roll_comp)

         # self.logger.info(f"roll_comp {self.pid_roll.deltaErr}")
         # self.logger.info(f"pitch_comp {pitch_comp.deltaErr}
         # self.logger.info(f"yaw {roll_comp.deltaErr}
         

         # print(angs)

         # Calculate motor speeds
         mSpeeds = [
            throttle_scaled + pitch_comp + roll_comp + yaw_comp,
            throttle_scaled - pitch_comp + roll_comp - yaw_comp,
            throttle_scaled - pitch_comp - roll_comp + yaw_comp,
            throttle_scaled + pitch_comp - roll_comp - yaw_comp
         ]

         # Output motor speeds
         if (rx_data[2] > self.throttle_cutoff):
            self.outputSpeeds(mSpeeds)
         else:
            self.motors.zero_throttle()
            resetError()
         # Read PID values from socket
         # self.readSocketPID()

         # Calculate drift
         # lateral_error = self.zed.get_pos_relative()
         # x_comp = 1000 * self.pid_roll.calc(0, lateral_error[0])
         # z_comp = 1000 * self.pid_yaw.calc(0, lateral_error[2])
         # # self.logger.info(f"dX: {x_comp}\tdZ: {z_comp}")

         # # Normalize throttle
         # throttle_norm = (rx_data[2] - 1000) / 1000

         # # Scale throttle
         # throttle_scaled = 100 * self.throttle_scale * throttle_norm

         # mSpeeds = [
         #    np.clip(int(throttle_scaled + x_comp + z_comp), 0, 100),
         #    np.clip(int(throttle_scaled + x_comp - z_comp), 0, 100),
         #    np.clip(int(throttle_scaled - x_comp + z_comp), 0, 100),
         #    np.clip(int(throttle_scaled - x_comp - z_comp), 0, 100)
         # ]

         # if (rx_data[2] > self.throttle_cutoff):
         #    self.motors.output_speeds(mSpeeds)
         # else:
         #    self.motors.zero_throttle()
         # pDelta = self.zed.get_position_diff()
         # angs = self.zed.get_euler()

         # pid_setpoints = [
         #     0.12 * rx_data[3] - 180,
         #     -0.03 * rx_data[1] + 45,
         #     -0.03 * rx_data[0] + 45
         # ]

         # out_speeds = [int(self.throttle_scale * rx_data[2]) + 500] * 4
         # self.motors.output_speeds(out_speeds)
         # sleep(1 / self.sample_rate)


   def close(self):
      self.motors.output_speeds([1000] * 4)
      self.sock.close()
      self.zed.closeCamera()
      print(f"\nP = {self.kP}\nI = {self.kI}\nD = {self.kD}")
      