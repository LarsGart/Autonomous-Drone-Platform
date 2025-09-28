import serial
import numpy as np
from collections import namedtuple
from ibus import IBus

RCChannels = namedtuple('RCChannels', ['roll', 'pitch', 'throttle', 'yaw'])


UART_PORT = '/dev/ttyTHS1'
UART_BAUDRATE = 115200

CHANNEL_MIN = 1000
CHANNEL_MAX = 2000
CHANNEL_CENTER = 1500
CHANNEL_NORM = 0.002
CHANNEL_OFFSET = -3.0
CHANNEL_DEADBAND = 8
MISSED_READINGS_THRESHOLD = 100

FAILSAFE = RCChannels(roll=1500, pitch=1500, throttle=1000, yaw=1500)
CHANNEL_COUNT = 14


class Receiver:
    '''Handles reading receiver input via IBUS over UART.'''

    def __init__(self) -> None:
        self.uart = serial.Serial(port=UART_PORT, baudrate=UART_BAUDRATE)
        self.ibus = IBus(self.uart)
        self.input: list[int] = [0] * CHANNEL_COUNT
        self.output: RCChannels = RCChannels(CHANNEL_CENTER, CHANNEL_CENTER, CHANNEL_CENTER, CHANNEL_CENTER)
        self.missed_readings: int = 0

    def _create_deadband(self, value: int) -> int:
        '''Apply deadband around joystick center.'''
        if CHANNEL_CENTER - CHANNEL_DEADBAND < value < CHANNEL_CENTER + CHANNEL_DEADBAND:
            return CHANNEL_CENTER
        return value

    def _read(self) -> RCChannels:
        '''Read receiver input; applies clipping, deadband, and failsafe if needed.'''
        self.uart.reset_input_buffer()
        self.input = self.ibus._read_frame()

        if self.input:
            clipped = np.clip(self.input, CHANNEL_MIN, CHANNEL_MAX)

            self.output = RCChannels(
                roll     = self._create_deadband(int(clipped[0])),
                pitch    = self._create_deadband(int(clipped[1])),
                throttle = int(clipped[2]),
                yaw      = self._create_deadband(int(clipped[3])),
            )
            self.missed_readings = 0
        else:
            self.missed_readings += 1
            if self.missed_readings > MISSED_READINGS_THRESHOLD:
                self.output = FAILSAFE
    
        return self.output

    def _read_normalized(self) -> RCChannels:
        '''
        Read normalized RC input directional components in range ~[-1.0, 1.0]).
        Throttle is normalized to [0.0, 1.0].
        '''
        channels = self._read()
        return RCChannels(
            roll=CHANNEL_NORM * channels.roll + CHANNEL_OFFSET,
            pitch=CHANNEL_NORM * channels.pitch + CHANNEL_OFFSET,
            throttle=CHANNEL_NORM / 2 * channels.throttle - 1,
            yaw=CHANNEL_NORM * channels.yaw + CHANNEL_OFFSET,
        )

    def __del__(self) -> None:
        try:
            if self.uart and self.uart.is_open:
                self.uart.close()
        except AttributeError:
            pass
