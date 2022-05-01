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
Date Created: April 19, 2022
"""

from smbus2 import SMBus
from time import sleep

class OrienationSensor:

    # This function initializes the acceleromter, magnetomer, and gyroscope
    def initSensor(self, I2CBus):
        self.bus = SMBus(I2CBus)

        '''
        Accelerometer + Magnetometer initialization
        '''

        addr = 0x1F # I2C Address of the accelerometer + magnetometer

        # Read the WHO_AM_I register to make sure the sensor is correct
        assert (self.bus.read_byte_data(addr, 0x0D) == 0xC7), "This sensor is sus!"

        # Write 0x00 (0000 0000) to CTRL_REG1 to place sensor in standby mode
        self.bus.write_byte_data(addr, 0x2A, 0x00)

        # Write 0x02 (0000 0010) to CTRL_REG2 to set accel mode to high resolution
        self.bus.write_byte_data(addr, 0x2B, 0x02)

        # Write 0x1F (0001 1111) to M_CTRL_REG1 to put mag in hybrid mode and enable 8x oversampling
        self.bus.write_byte_data(addr, 0x5B, 0x1F)

        # Write 0x20 (0010 0000) to M_CTRL_REG2 to put let mag output registers follow accel out registers
        self.bus.write_byte_data(addr, 0x5C, 0x20)

        # Write 0x01 (0000 0001) to XYZ_DATA_CFG to set accel range to +/-4g
        self.bus.write_byte_data(addr, 0x0E, 0x01)

        # Write 0x15 (0001 0101) to CTRL_REG1 to set ODR to 100Hz and place sensor in active mode
        self.bus.write_byte_data(addr, 0x2A, 0x15)

        '''
        Gyroscope initialization
        '''

        addr = 0x21 # I2C Address of the gyroscope

        # Read the WHO_AM_I register to make sure the sensor is correct
        assert (self.bus.read_byte_data(addr, 0x0C) == 0xD7), "This sensor is sus!"

        # Write 0x00 (0000 0000) to CTRL_REG1 to place sensor in standby mode
        self.bus.write_byte_data(addr, 0x13, 0x00)

        # Write 0x03 (0000 0011) to CTRL_REG0 to set sensitivity to +/-250dps
        self.bus.write_byte_data(addr, 0x0D, 0x03)

        # Write 0x0E (0000 1110) to CTRL_REG1 to set ODR to 100Hz and to place sensor in active mode
        self.bus.write_byte_data(addr, 0x13, 0x0E)

        # Wait 60ms + 1/ODR for sensor to transition from standby to active mode
        sleep(0.1)

    # This function reads the sensor readings and outputs them
    def readSensor(self):
        # TODO: write code lol
        pass