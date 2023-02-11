'''
Author: Lars Gartenberg

This module contains two main classes, TimestampHandler and ZedModel.

The TimestampHandler class is responsible for handling
the timestamps of various sensors and determining if the data is new or old.
It has three instance variables t_imu, t_baro and t_mag which store the timestamps
for the IMU, barometer and magnetometer respectively.
The is_new method is used to determine if a new timestamp is
greater than the stored reference and updates it if necessary.

The ZedModel class has methods to open and close the camera,
get camera and sensor configuration information,
and log information to a file
'''
import pyzed.sl as sl

import cv2
import numpy as np
import math
import serial
import logging


'''
Class to handle timestamps of various sensors and determine if data is new or old.
'''
class TimestampHandler:
    def __init__(self):
        self.t_imu = sl.Timestamp()
        self.t_baro = sl.Timestamp()
        self.t_mag = sl.Timestamp()
    
    '''
    Determines if a new timestamp is greater than the stored reference and updates it if necessary.
    '''
    def is_new(self, sensor):
        if (isinstance(sensor, sl.IMUData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_imu.get_microseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_
        elif (isinstance(sensor, sl.MagnetometerData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_mag.get_microseconds())
            if new_:
                self.t_mag = sensor.timestamp
            return new_
        elif (isinstance(sensor, sl.BarometerData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_baro.get_microseconds())
            if new_:
                self.t_baro = sensor.timestamp
            return new_


'''
Class to instantiate a Zed camera object.
'''
class ZedModel():
    '''
    Constructor for ZedModel class.
    Initializes the Zed camera object and sets the depth mode to NONE.
    '''
    def __init__(self):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.NONE
        self.info = self.zed.get_camera_information()
        self.cam_model = self.info.camera_model
        self.sensors = ['accelerometer', 'gyroscope', 'magnetometer', 'barometer']
        self.ts_handler = TimestampHandler()
        logging.basicConfig(filename='zed_model.log'
                           ,filemode='w'
                           ,level = logging.DEBUG
                           ,format='%(asctime)s:%(levelname)s:%(message)s')
        self.logger = logging.getLogger()

    def openCamera(self):
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.logger.warning(repr(err))
            self.zed.close()
            exit(1)
        return True
    

    def closeCamera(self):
        if self.zed.is_opened():
            try:
                self.zed.close()
                self.logger.info("Camera closed")
            except:
                self.logger.warning("Camera could not be closed")
        else:
            self.logger.warning("Camera not open")
    

    def get_camera_configuration(self):
        self.logger.info(self.info)
        self.logger.info("Camera Model: " + str(self.cam_model))
        self.logger.info("Serial Number: " + str(self.info.serial_number))
        self.logger.info("Camera Firmware: " + str(self.info.camera_configuration.firmware_version))
        self.logger.info("Sensors Firmware: " + str(self.info.sensors_configuration.firmware_version))

        return self.info

    '''
    Retrieves configuration information for each sensor in the `sensors` list and logs the following information:
    - Sensor type
    - Maximum rate
    - Range
    - Resolution
    - Noise density (if available)
    - Random walk (if available)
    Returns the sensor configuration information for the first available sensor in the list.
    '''
    def get_sensor_configuration(self):
        for sensor in self.sensors:
            sensor_field = 'sensors_' + sensor + '_configuration'
            sensor_config = self.info.__dict__[sensor_field]
            if sensor_config.is_available:
                self.logger.info("Sensor type: " + str(sensor_config.sensor_type))
                self.logger.info("Max rate: " + str(sensor_config.sampling_rate) + " " + str(sl.SENSORS_UNIT.HERTZ))
                self.logger.info("Range: " + str(sensor_config.sensor_range) + " " + str(sensor_config.sensor_unit))
                self.logger.info("Resolution: " + str(sensor_config.resolution) + " " + str(sensor_config.sensor_unit))
                try:
                    self.logger.info("Noise Density: " + str(sensor_config.noise_density) + " " + str(sensor_config.sensor_unit) + "Hz")
                    self.logger.info("Random Walk: " + str(sensor_config.random_walk) + " " + str(sensor_config.sensor_unit) + "Hz")
                except:
                    self.logger.warning("NaN values for noise density and/or random walk")
                return sensor_config
            
    '''
    Retrieve sensor data from the ZED camera based on the specified sensor type.

    Args:
    sensor_type (str, optional): Specify the type of sensor data to retrieve. 
                                Options include 'accelerometer', 'gyroscope', 'magnetometer', 'barometer', and 'all'.
                                Default value is None.

    Returns:
    Depending on the `sensor_type`, the function returns:
        - A list of linear acceleration values in [m/sec^2] if the `sensor_type` is 'accelerometer'.
        - A list of angular velocity values in [deg/sec] if the `sensor_type` is 'gyroscope'.
        - A list of calibrated magnetic field values in [uT] if the `sensor_type` is 'magnetometer'.
        - The pressure value in [hPa] if the `sensor_type` is 'barometer'.
        - A dictionary of all sensor data if the `sensor_type` is 'all'.

    Raises:
    Warning: If the specified `sensor_type` is not available or the data has not been updated.
    '''
    def get_sensor_data(self, sensor_type=None):
        self.timestamp_handler = TimestampHandler()
        self.sensors_data = sl.SensorsData()
        # Checks if the sensors_data is available and if the timestamp is new
        if self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) != sl.ERROR_CODE.SUCCESS:
            self.logger.warning("Sensor data not available because the ZED is not opened")
            return None

        if not self.timestamp_handler.is_new(self.sensors_data.get_imu_data()):
            self.logger.warning("Data not updated")
            return None

        data = {}
        if sensor_type == 'all':
            for sensor in ['accelerometer', 'gyroscope', 'magnetometer', 'barometer']:
                data[sensor] = self.get_sensor_data(sensor)
        elif sensor_type == 'accelerometer':
            linear_acceleration = self.sensors_data.get_imu_data().get_linear_acceleration()
            self.logger.info(" \t Acceleration: [ {0} {1} {2} ] [m/sec^2]".format(linear_acceleration[0], 
                                                                                linear_acceleration[1], 
                                                                                linear_acceleration[2]))
            data = linear_acceleration
        elif sensor_type == 'gyroscope':
            angular_velocity = self.sensors_data.get_imu_data().get_angular_velocity()
            self.logger.info(" \t Angular Velocities: [ {0} {1} {2} ] [deg/sec]".format(angular_velocity[0], 
                                                                                        angular_velocity[1], 
                                                                                        angular_velocity[2]))
            data = angular_velocity
        elif sensor_type == 'magnetometer':
            if not self.timestamp_handler.is_new(self.sensors_data.get_magnetometer_data()):
                self.logger.warning("Magnetometer data has not been updated")
                return None
            magnetic_field_calibrated = self.sensors_data.get_magnetometer_data().get_calibrated_magnetic_field()
            self.logger.info(" \t Magnetic Field: [ {0} {1} {2} ] [uT]".format(magnetic_field_calibrated[0], 
                                                                            magnetic_field_calibrated[1], 
                                                                            magnetic_field_calibrated[2]))
            data = magnetic_field_calibrated
        elif sensor_type == 'barometer':
            if not self.timestamp_handler.is_new(self.sensors_data.get_barometer_data()):
                self.logger.warning("Barometer data has not been updated")
                return None
            pressure = self.sensors_data.get_barometer_data().pressure()
            self.logger.info(" \t Pressure: {0} [hPa]".format(pressure))
            data = pressure
        else:
            self.logger.warning("Sensor type not available")
            return None

        return data

    '''
    Retrieves the orientation data from the IMU sensor using the Zed Camera API and returns the filtered orientation quaternion.
    The orientation data is represented as a 4-element quaternion in the form [Ox, Oy, Oz, Ow].

    Returns:
    List[float]: A list of 4 float values representing the orientation quaternion.

    Raises:
    Warning: If the IMU data has not been updated.
    '''
    def get_quaternion(self):
        self.timestamp_handler = TimestampHandler()
        self.sensors_data = sl.SensorsData()

        # Checks if the sensors_data is available and if the timestamp is new
        if self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS :
            if self.timestamp_handler.is_new(self.sensors_data.get_imu_data()):
                # Filtered orientation quaternion
                quaternion = self.sensors_data.get_imu_data().get_pose().get_orientation().get()
                self.logger.info(" \t Orientation: [ Ox: {0}, Oy: {1}, Oz {2}, Ow: {3} ]".format(quaternion[0], 
                                                                                                 quaternion[1], 
                                                                                                 quaternion[2], 
                                                                                                 quaternion[3]))
                return quaternion
            else:
                self.logger.warning("IMU data has not been updated")
                return None
