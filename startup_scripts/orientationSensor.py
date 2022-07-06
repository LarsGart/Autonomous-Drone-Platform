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

from time import sleep
import numpy as np
import ctypes
import pylibi2c

_ADDR_ACCELMAG = 0x1F
_ADDR_GYRO = 0x21

class OrientationSensor:

    # Convert an unsigned number to signed using twos complement
    def __twosComp(self, val, bits):
        if val & (1 << (bits - 1)) != 0:
            return val - (1 << bits)
        return val

    # Initialize the acceleromter, magnetomer, and gyroscope
    def __init__(self, bus):
        self.accelmag = pylibi2c.I2CDevice(bus, _ADDR_ACCELMAG)
        self.gyro = pylibi2c.I2CDevice(bus, _ADDR_GYRO)

        '''
        Accelerometer + Magnetometer initialization
        '''

        # Read the WHO_AM_I register to make sure the sensor is correct
        assert (self.accelmag.ioctl_read(0x0D, 1) == b'\xC7'), "This sensor is sus!"

        # Write 0x00 (0000 0000) to CTRL_REG1 to place sensor in standby mode
        self.accelmag.ioctl_write(0x2A, b'\x00')

        # Write 0x02 (0000 0010) to CTRL_REG2 to set accel mode to high resolution
        self.accelmag.ioctl_write(0x2B, b'\x02')

        # Write 0x1F (0001 1111) to M_CTRL_REG1 to put mag in hybrid mode and enable 8x oversampling
        self.accelmag.ioctl_write(0x5B, b'\x1F')

        # Write 0x20 (0010 0000) to M_CTRL_REG2 to put let mag output registers follow accel out registers
        self.accelmag.ioctl_write(0x5C, b'\x20')

        # Write 0x01 (0000 0001) to XYZ_DATA_CFG to set accel range to +/-4g
        self.accelmag.ioctl_write(0x0E, b'\x01')

        # Write 0x15 (0001 0101) to CTRL_REG1 to set ODR to 100Hz and place sensor in reduced noise and active mode
        self.accelmag.ioctl_write(0x2A, b'\x15')

        '''
        Gyroscope initialization
        '''

        # Read the WHO_AM_I register to make sure the sensor is correct
        assert (self.gyro.ioctl_read(0x0C, 1) == b'\xD7'), "This sensor is sus!"

        # Write 0x00 (0000 0000) to CTRL_REG1 to place sensor in standby mode
        self.gyro.ioctl_write(0x13, b'\x00')

        # Write 0x03 (0000 0011) to CTRL_REG0 to set sensitivity to +/-250dps
        self.gyro.ioctl_write(0x0D, b'\x03')

        # Write 0x0E (0000 1110) to CTRL_REG1 to set ODR to 100Hz and to place sensor in active mode
        self.gyro.ioctl_write(0x13, b'\x0E')

        # Wait 60ms + 1/ODR for sensor to transition from standby to active mode
        sleep(0.1)
    
    # Close the busses when the sensor is deleted
    def __del__(self):
        self.accelmag.close()
        self.gyro.close()

    # Read the sensor readings and output them
    def readSensor(self):
        # Read 13 bytes (1 status byte + 6 accel bytes + 6 mag bytes) from accel + mag
        rawAccelMagData = self.accelmag.ioctl_read(0x00, 13)

        # Convert raw accel data to signed real world values
        accelData = 0.000488 * 9.80665 * np.array([ # 0.488mg / LSB
            self.__twosComp(((rawAccelMagData[1] << 8) | rawAccelMagData[2]) >> 2, 14),
            self.__twosComp(((rawAccelMagData[3] << 8) | rawAccelMagData[4]) >> 2, 14),
            self.__twosComp(((rawAccelMagData[5] << 8) | rawAccelMagData[6]) >> 2, 14)
        ])

        # Convert raw mag data to signed real world values
        magData = 0.1 * np.array([ # 0.1uT / LSB 
            self.__twosComp((rawAccelMagData[7] << 8) | rawAccelMagData[8], 16),
            self.__twosComp((rawAccelMagData[9] << 8) | rawAccelMagData[10], 16),
            self.__twosComp((rawAccelMagData[11] << 8) | rawAccelMagData[12], 16)
        ])

        # Read 7 bytes (1 status byte + 6 gyro bytes) from gyro
        rawGyroData = self.gyro.ioctl_read(0x00, 7)

        # Convert raw gyro data to signed real world values
        gyroData = 0.0078125 * np.array([ # 7.8125mdps / LSB
            self.__twosComp((rawGyroData[1] << 8) | rawGyroData[2], 16),
            self.__twosComp((rawGyroData[3] << 8) | rawGyroData[4], 16),
            self.__twosComp((rawGyroData[5] << 8) | rawGyroData[6], 16)
        ])

        return accelData, magData, gyroData