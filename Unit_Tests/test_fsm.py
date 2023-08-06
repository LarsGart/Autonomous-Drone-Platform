import unittest
import serial

from Models.flight_controller_model import FlightController

class TestUART:
    def __init__(self):
        self.uart = serial.Serial(port="/dev/ttyS0", baudrate=115200)

    def read(self):
        return self.uart.read()

    def write(self, data):
        self.uart.write(data)

class TestFSM(unittest.TestCase):
    def setUp(self):
        self.flight_controller = FlightController()
        self.uart = TestUART()
        