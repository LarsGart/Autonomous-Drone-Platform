import spidev

# SPI configuration info
SPI_BUS = 0                 # spidev0
SPI_SS  = 0                 # spidev0.0
SPI_CLOCK = 4000000         # 4 Mhz baudrate

# SPI transaction info
START_BYTES = [0xBC, 0x9E]  # Start bytes of transaction
READ_IDLE_BYTES = 4 # Number of idle bytes inserted between a read request and the read data

# Command list
cmd_dict = {
  'SET_ARM_STATUS'    : 0x00,
  'SET_MOTOR_SPEEDS'  : 0x01,
  'STOP_MOTORS'       : 0x02,
  'READ_REGISTER'     : 0x03
}

# Register list
# [register address, data length]
reg_dict = {
  'FIRMWARE_VERSION'  : {'addr':0x00, 'length':2},
  'BATTERY_VOLTAGE'   : {'addr':0x01, 'length':2},
  'BATTERY_CURRENT'   : {'addr':0x02, 'length':2},
  'TELEMETRY_DATA'    : {'addr':0x03, 'length':6},
  'ARM_STATUS'        : {'addr':0x04, 'length':1},
  'MOTOR_SPEEDS'      : {'addr':0x05, 'length':8}
}

class Xiao:
  def __init__(self):
    self.spi = spidev.SpiDev(SPI_BUS, SPI_SS)
    self.spi.max_speed_hz = SPI_CLOCK
    self.spi.mode = 0

  def _compute_crc(self, data: list) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
  
  def _send_data(self, cmd, bytes):
    crc16 = self.__compute_crc(START_BYTES + [cmd] + bytes)
    self.spi.xfer(START_BYTES + [cmd] + bytes + [crc16 >> 8 & 0xFF, crc16 & 0xFF])

  def _read_register(self, reg):
    self._send_data(cmd_dict['READ_REGISTER'], [reg['addr']])
    reg_data = self.spi.xfer((reg['length'] + READ_IDLE_BYTES)*[0xFF])
    reg_data = reg_data[4:]
    return reg_data

  @property
  def _esc_arm(self):
    self._send_data(cmd_dict['SET_ARM_STATUS'], [0x01])

  @property
  def _esc_disarm(self):
    self._send_data(cmd_dict['SET_ARM_STATUS'], [0x00])

  def _esc_update_motors(self, speeds: list):
    byte_stream = [speed >> i & 0xFF for speed in speeds for i in (8, 0)]
    self._send_data(cmd_dict['SET_MOTOR_SPEEDS'], byte_stream)

  @property
  def _esc_stop_motors(self):
    self._send_data(cmd_dict['STOP_MOTORS'], [0xFF])

  @property
  def _read_firmware_version(self):
    return self._read_register(reg_dict['FIRMWARE_VERSION'])
  
  @property
  def _read_battery_voltage(self):
    return self._read_register(reg_dict['BATTERY_VOLTAGE'])
  
  @property
  def _read_battery_current(self):
    return self._read_register(reg_dict['BATTERY_CURRENT'])
  
  @property
  def _read_telemetry(self):
    return self._read_register(reg_dict['TELEMETRY_DATA'])

  @property
  def _read_arm_status(self):
    return self._read_register(reg_dict['ARM_STATUS'])

  @property
  def _read_motor_speeds(self):
    return self._read_register(reg_dict['MOTOR_SPEEDS'])
