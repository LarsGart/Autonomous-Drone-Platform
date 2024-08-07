'''
Author: Jerin Abraham

This class handles outputting motor speeds to the motor controller MCU
'''

import serial
import random
import numpy as np


class Motors:
   '''
   Instantiates the UART and connects to the MCU
   '''
   def __init__(self):
      # logging.basicConfig(filename=f"/home/drone/Autonomous-Drone-Platform/Logs/{self.__class__.__name__}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
      #                      ,level=logging.DEBUG
      #                      ,format='%(asctime)s:%(levelname)s:%(message)s')
      # self.logger = logging.getLogger()
      # Define object variables
      self.test_passed = False
      self.test_range = 100
      self.num_motors = 4

      # Connect to the MCU
      self.connect()

   '''
   Tests the connection and speed transmission between the Jetson and the MCU
   '''
   def test_motors(self):
      print("Testing Motors")
      # Send 100 test speeds to verify connection and encryption/decryption
      for _ in range(self.test_range):
         test_speeds = [random.randint(0, 100) for x in range(self.num_motors)]
         self.output_speeds(test_speeds)
         speed_list_unscaled = [x for x in self.uart.read_until().decode().strip().split(",")]
         speed_list = [int((int(x) - 1000) / 10) for x in speed_list_unscaled]
         for i in range(self.num_motors):
               recvSpd = speed_list[i]
               calcSpd = test_speeds[i]
               if (calcSpd != recvSpd):
                  print(f"Test Failed: Expected {self.scale_speeds(test_speeds)}, Received {speed_list}")
                  self.uart.write(bytearray([21])) # Send NACK
                  
                  return False
               
      # End testing and set MCU into receiveSpeed state
      self.uart.write(bytearray([6])) # Send ACK
      self.uart.write(bytearray([4])) # Send EOT
      print('Motor tests passed')

      # Set motors to 0 speed and return True to indicate the motors have passed testing
      self.zero_throttle()
      self.test_passed = True
      return True

   '''
   Disconnects from the MCU upon deletion
   '''
   def __del__(self):
      self.disconnect()

   '''
   Connects to the MCU and ACK's it to move it into the testing state
   '''
   def connect(self):
      print("Attempting to connect to MCU. Waiting for ENQ...")
      self.uart = serial.Serial(port="/dev/ttyS0", baudrate=115200)

      # Wait for an ENQ
      while self.uart.read() != b'\x05':
         pass
      print("ENQ recieved. Connecting to MCU")
      self.uart.write(bytearray([6])) # Send ACK

   '''
   Disconnects from the MCU and closes the UART
   '''
   def disconnect(self):
      if self.uart:
         # Send an ESC to set the state machine back to the waitForConnection state and close the UART
         self.uart.write(bytearray([27]))
         self.uart.close()
         del self.uart

   '''
   Sends data over UART
   '''
   def send_data(self, data):
      if self.test_passed:
         encoded = None
         if (type(data) == str and len(data) == 1):
            encoded = data.encode()
            self.uart.write(encoded)
         elif type(data) == int:
            encoded = bytearray([data])
            self.uart.write(encoded)
         return encoded

   '''
   Splits speeds into MSB and LSB for each speed and send byte stream to speed controller via UART
   '''
   def output_speeds(self, speeds: list):
      if self.validate_speeds(speeds):
         scaled_speeds = self.scale_speeds(speeds)
         byte_stream = [2] + [speed >> i & 255 for speed in scaled_speeds for i in (8, 0)] # Sends a '<' and '>'.
         self.uart.write(bytearray(byte_stream))
         return byte_stream
      
   '''
   Sends zero throttle data to the motors
   '''
   def zero_throttle(self):
      self.output_speeds([0] * 4)
    
   def validate_speeds(self, speeds):
      if len(speeds) != 4:
         return False
      for speed in speeds:
         if (speed < 0 or speed > 100):
            return False
      return True

   def scale_speeds(self, speeds): # Scaling and constraining speeds from 0-100 to 1000-2000
      scaled_speeds = [np.clip(int(speed * 10 + 1000), 1000, 2000) for speed in speeds]
      print(scaled_speeds)
      return scaled_speeds