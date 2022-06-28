#!/usr/bin/env python
"""
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

This scripts communicates with the onboard orientation sensor via I2C in order to
extract 

This class will:
    -initialize the onboard orientation sensor
    -read the accelerometer, gyroscope, and magnetometer values

The following datasheets were used to write this script
https://www.nxp.com/docs/en/data-sheet/FXAS21002.pdf
https://www.nxp.com/docs/en/data-sheet/FXOS8700CQ.pdf

Author: Jerin Abraham
"""

from smbus2 import SMBus
from time import sleep

# I2C addresses of the sensors
ADDR_ACCELMAG = 0x1F
ADDR_GYRO = 0x21

class OrienationSensor:

    # This initializes the acceleromter, magnetomer, and gyroscope
    def __init__(self, I2CBus):
        self.bus = SMBus(I2CBus)

        '''
        Accelerometer + Magnetometer initialization
        '''

        # Read the WHO_AM_I register to make sure the sensor is correct
        assert (self.bus.read_byte_data(ADDR_ACCELMAG, 0x0D) == 0xC7), "This sensor is sus!"

        # Write 0x00 (0000 0000) to CTRL_REG1 to place sensor in standby mode
        self.bus.write_byte_data(ADDR_ACCELMAG, 0x2A, 0x00)

        # Write 0x02 (0000 0010) to CTRL_REG2 to set accel mode to high resolution
        self.bus.write_byte_data(ADDR_ACCELMAG, 0x2B, 0x02)

        # Write 0x1F (0001 1111) to M_CTRL_REG1 to put mag in hybrid mode and enable 8x oversampling
        self.bus.write_byte_data(ADDR_ACCELMAG, 0x5B, 0x1F)

        # Write 0x20 (0010 0000) to M_CTRL_REG2 to put let mag output registers follow accel out registers
        self.bus.write_byte_data(ADDR_ACCELMAG, 0x5C, 0x20)

        # Write 0x01 (0000 0001) to XYZ_DATA_CFG to set accel range to +/-4g
        self.bus.write_byte_data(ADDR_ACCELMAG, 0x0E, 0x01)

        # Write 0x15 (0001 0101) to CTRL_REG1 to set ODR to 100Hz and place sensor in active mode
        self.bus.write_byte_data(ADDR_ACCELMAG, 0x2A, 0x15)

        '''
        Gyroscope initialization
        '''

        # Read the WHO_AM_I register to make sure the sensor is correct
        assert (self.bus.read_byte_data(ADDR_GYRO, 0x0C) == 0xD7), "This sensor is sus!"

        # Write 0x00 (0000 0000) to CTRL_REG1 to place sensor in standby mode
        self.bus.write_byte_data(ADDR_GYRO, 0x13, 0x00)

        # Write 0x03 (0000 0011) to CTRL_REG0 to set sensitivity to +/-250dps
        self.bus.write_byte_data(ADDR_GYRO, 0x0D, 0x03)

        # Write 0x0E (0000 1110) to CTRL_REG1 to set ODR to 100Hz and to place sensor in active mode
        self.bus.write_byte_data(ADDR_GYRO, 0x13, 0x0E)

        # Wait 60ms + 1/ODR for sensor to transition from standby to active mode
        sleep(0.1)

    # This function reads the sensor readings and outputs them
    def readSensor(self):
        # Read 13 bytes (1 status byte + 6 accel bytes + 6 mag bytes) from accel + mag
        rawAccelMagData = self.bus.read_i2c_block_data(ADDR_ACCELMAG, 0x00, 13)

        # Read 7 bytes (1 status byte + 6 gyro bytes) from gyro
        rawGyroData = self.bus.read_i2c_block_data(ADDR_GYRO, 0x00, 7)

        # Convert data into readable format
        sensorData = [
            (((rawAccelMagData[1] << 8) | rawAccelMagData[2]) >> 2) * 0.000488 * 9.80665, # Accel x
            (((rawAccelMagData[3] << 8) | rawAccelMagData[4]) >> 2) * 0.000488 * 9.80665, # Accel y
            (((rawAccelMagData[5] << 8) | rawAccelMagData[6]) >> 2) * 0.000488 * 9.80665, # Accel z
            (rawAccelMagData[7] << 8) | rawAccelMagData[8], # Mag x
            (rawAccelMagData[9] << 8) | rawAccelMagData[10], # Mag y
            (rawAccelMagData[11] << 8) | rawAccelMagData[12], # Mag z
            (rawGyroData[1] << 8) | rawGyroData[2], # Gyro x
            (rawGyroData[3] << 8) | rawGyroData[4], # Gyro y
            (rawGyroData[5] << 8) | rawGyroData[6] # Gyro z
        ]

        return sensorData