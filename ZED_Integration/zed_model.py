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
Class to instantiate a Zed camera object.
'''
class ZedModel():
    '''
    Constructor for ZedModel class.
    Initializes the Zed camera object and sets the depth mode to NONE.
    '''
    def __init__(self):
        logging.basicConfig(filename='zed_model.log'
                           ,filemode='w'
                           ,level = logging.DEBUG
                           ,format='%(asctime)s:%(levelname)s:%(message)s')
        self.logger = logging.getLogger()

        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.NONE
        self.openCamera()
        self.info = self.zed.get_camera_information()
        self.cam_model = self.info.camera_model
        self.sensors = ['accelerometer', 'gyroscope']
        self.logger.info("ZedModel initialized")


    def openCamera(self):
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.logger.warning(repr(err))
            exit(1)
        self.logger.info("Camera opened")
        return True


    def closeCamera(self):
        if self.zed.is_opened():
            self.zed.close()
            self.logger.info("Camera closed")
        else:
            self.logger.warning("Camera not open")


    def get_camera_configuration(self):
        camera_config = self.info
        self.logger.info(camera_config)
        self.logger.info(f"Camera Model: {self.cam_model}")
        self.logger.info(f"Serial Number: {camera_config.serial_number}")
        self.logger.info(f"Camera Firmware: {camera_config.camera_configuration.firmware_version}")
        self.logger.info(f"Sensors Firmware: {camera_config.sensors_configuration.firmware_version}")
        return camera_config


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
            sensor_config = getattr(self.info.sensors_configuration, f'{sensor}_parameters')
            if sensor_config.is_available:
                self.logger.info(f"Sensor type: {sensor_config.sensor_type}")
                self.logger.info(f"Max rate: {sensor_config.sampling_rate} {sl.SENSORS_UNIT.HERTZ}")
                self.logger.info(f"Range: {sensor_config.sensor_range} {sensor_config.sensor_unit}")
                self.logger.info(f"Resolution: {sensor_config.resolution} {sensor_config.sensor_unit}")
                try:
                    self.logger.info(f"Noise Density: {sensor_config.noise_density} {sensor_config.sensor_unit}Hz")
                    self.logger.info(f"Random Walk: {sensor_config.random_walk} {sensor_config.sensor_unit}Hz")
                except TypeError: # this will 
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
        - A dictionary of all sensor data if the `sensor_type` is 'all'.

    Raises:
    Warning: If the specified `sensor_type` is not available or the data has not been updated.
    '''
    def get_sensor_data(self, sensor_type=None):
        self.sensors_data = sl.SensorsData()
        if self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) != sl.ERROR_CODE.SUCCESS:
            self.logger.warning("Sensor data not available because the ZED is not opened")
            return None

        data = {}
        if sensor_type == 'all':
            for sensor in self.sensors:
                data[sensor] = self.get_sensor_data(sensor)
        elif sensor_type == 'accelerometer':
            linear_acceleration = self.sensors_data.get_imu_data().get_linear_acceleration()
            self.logger.info(" \t Acceleration: [ {0} {1} {2} ] [m/sec^2]".format(*linear_acceleration))
            data = linear_acceleration
        elif sensor_type == 'gyroscope':
            angular_velocity = self.sensors_data.get_imu_data().get_angular_velocity()
            self.logger.info(" \t Angular Velocities: [ {0} {1} {2} ] [deg/sec]".format(*angular_velocity))
            data = angular_velocity
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
        self.sensors_data = sl.SensorsData()

        if self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            # Get filtered orientation quaternion
            quaternion = self.sensors_data.get_imu_data().get_pose().get_orientation().get()
            self.logger.info(" \t Orientation: [ Ox: {0}, Oy: {1}, Oz {2}, Ow: {3} ]".format(*quaternion)) # the * is the 
            return quaternion
        else:
            self.logger.warning("IMU data has not been updated")
            return None

    def get_euler(self):
        pass #TODO: make this https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
