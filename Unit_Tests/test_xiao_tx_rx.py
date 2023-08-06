import random
import unittest

from Models.motor_model import Motors


class TestMotors(unittest.TestCase):
    def setUp(self):
        self.motor_model = Motors()
        self.motor_speed_range = range(1000, 2000)

    def test_send_data(self):
        test_char = self.motor_model.sendData('a')
        self.assertEqual(test_char, b'a')

        test_int = self.motor_model.sendData(1)
        self.assertEqual(test_int, bytearray(b'\x01'))

    def test_ranomized_output_speeds(self):
        for _ in range(10):
            speeds = [random.choice(self.motor_speed_range) for _ in range(4)]
            values = self.motor_model.outputSpeeds(speeds)
            self.assertEqual(values, [60] + [speed >> i & 255 for speed in test_speeds for i in (8, 0)] + [62])

    def test_invalid_speeds(self):
        speeds = [random.randint(-100, 0) for _ in range(4)]
        values = self.motor_model.outputSpeeds(speeds)
        self.assertEqual(values, None)

        speeds = [random.randint(0, 100) for _ in range(2)]
        values = self.motor_model.outputSpeeds(speeds)
        self.assertEqual(values, None)
        