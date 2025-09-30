import spidev

# ----------------- SPI Configuration -----------------
SPI_BUS = 0                 # spidev0
SPI_SS = 0                  # spidev0.0
SPI_CLOCK = 4_000_000       # 4 MHz

START_BYTES = [0xBC, 0x9E]  # Start bytes of transaction
READ_IDLE_BYTES = 4         # Idle bytes between request and response

# ----------------- Command List -----------------
CMD = {
    'SET_ARM_STATUS':   0x00,
    'SET_MOTOR_SPEEDS': 0x01,
    'STOP_MOTORS':      0x02,
    'READ_REGISTER':    0x03,
}

# ----------------- Register Map -----------------
REG = {
    'FIRMWARE_VERSION': {'addr': 0x00, 'length': 2},
    'BATTERY_VOLTAGE':  {'addr': 0x01, 'length': 2},
    'BATTERY_CURRENT':  {'addr': 0x02, 'length': 2},
    'TELEMETRY_DATA':   {'addr': 0x03, 'length': 6},
    'ARM_STATUS':       {'addr': 0x04, 'length': 1},
    'MOTOR_SPEEDS':     {'addr': 0x05, 'length': 8},
}

class Xiao:

    def __init__(self, bus: int = SPI_BUS, device: int = SPI_SS, clock: int = SPI_CLOCK):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = clock
        self.spi.mode = 0

    def __del__(self):
        self.spi.close()

    @staticmethod
    def _compute_crc(data: list[int]) -> int:
        '''Compute CRC-16-CCITT over a byte stream.'''
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def _send(self, cmd: int, payload: list[int]):
        '''Send a command + payload with CRC.'''
        frame = START_BYTES + [cmd] + payload
        crc16 = self._compute_crc(frame)
        frame += [crc16 >> 8 & 0xFF, crc16 & 0xFF]
        self.spi.xfer(frame)

    def _read_register(self, reg: dict) -> list[int]:
        '''Send read request and fetch register data.'''
        self._send(CMD['READ_REGISTER'], [reg['addr']])
        raw = self.spi.xfer([0xFF] * (reg['length'] + READ_IDLE_BYTES))
        return raw[READ_IDLE_BYTES:]  # strip idle bytes

    def _arm(self):
        self._send(CMD['SET_ARM_STATUS'], [0x01])

    def _disarm(self):
        self._send(CMD['SET_ARM_STATUS'], [0x00])

    def _stop_motors(self):
        self._send(CMD['STOP_MOTORS'], [0xFF])

    def _set_speeds(self, speeds: list[int]):
        # Remap motors to go clockwise around drone starting at front right corner
        reordered_speeds = [
            speeds[1],
            speeds[0],
            speeds[2],
            speeds[3]
        ]
        payload = []
        for speed in reordered_speeds:
            payload += [(speed >> 8) & 0xFF, speed & 0xFF]
        self._send(CMD['SET_MOTOR_SPEEDS'], payload)

    def _firmware_version(self) -> list[int]:
        return self._read_register(REG['FIRMWARE_VERSION'])

    def _battery_voltage(self) -> list[int]:
        return self._read_register(REG['BATTERY_VOLTAGE'])

    def _battery_current(self) -> list[int]:
        return self._read_register(REG['BATTERY_CURRENT'])

    def _telemetry(self) -> list[int]:
        return self._read_register(REG['TELEMETRY_DATA'])

    def _arm_status(self) -> list[int]:
        return self._read_register(REG['ARM_STATUS'])

    def _motor_speeds(self) -> list[int]:
        return self._read_register(REG['MOTOR_SPEEDS'])
