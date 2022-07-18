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


import numpy as np
import pylibi2c
from struct import unpack
from time import sleep

_ADDR_ACCELMAG = 0x1F # I2C address of the FXOS8700CQ
_ADDR_GYRO = 0x21 # I2C address of the FXAS21002

_ACCEL_4G_CONV = 0.000488 # 0.488mg / LSB
_MAG_1200UT_CONV = 0.1 # 0.1uT / LSB
_GYRO_250DPS_CONV = 0.0078125 # 7.8125mdps / LSB
_EARTH_GRAVITY = 9.80655 # in m/s^2

_ACCEL_CONV = _ACCEL_4G_CONV * _EARTH_GRAVITY # Increase performance by 1000% by calculating this beforehand

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

        # Convert raw accel and mag data to signed real world values
        accelData = np.multiply(_ACCEL_CONV, np.array([
            self.__twosComp(((rawAccelMagData[1] << 8) | rawAccelMagData[2]) >> 2, 14),
            self.__twosComp(((rawAccelMagData[3] << 8) | rawAccelMagData[4]) >> 2, 14),
            self.__twosComp(((rawAccelMagData[5] << 8) | rawAccelMagData[6]) >> 2, 14)
        ]))

        magData = np.multiply(_MAG_1200UT_CONV, np.array([
            unpack('>h', rawAccelMagData[7:9])[0],
            unpack('>h', rawAccelMagData[9:11])[0],
            unpack('>h', rawAccelMagData[11:13])[0]
        ]))

        # Read 7 bytes (1 status byte + 6 gyro bytes) from gyro
        rawGyroData = self.gyro.ioctl_read(0x00, 7)

        # Convert raw gyro data to signed real world values
        gyroData = np.multiply(_GYRO_250DPS_CONV, np.array([
            unpack('>h', rawGyroData[1:3])[0],
            unpack('>h', rawGyroData[3:5])[0],
            unpack('>h', rawGyroData[5:7])[0]
        ]))

        return accelData, magData, gyroData