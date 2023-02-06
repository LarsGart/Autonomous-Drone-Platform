'''
Condensing testMotorsandSensors.py
'''
import pyzed.sl as sl

import cv2
import numpy as np
import math
import serial
import logging


logging.basicConfig(filename='zed_model.log', 
                    filemode='w', 
                    level = logging.DEBUG, 
                    format='%(asctime)s:%(levelname)s:%(message)s')

'''
Basic class to handle the timestamp of the different sensors
to know if it is a new sensors_data or an old one
'''
class TimestampHandler:
    def __init__(self):
        self.t_imu = sl.Timestamp()
        self.t_baro = sl.Timestamp()
        self.t_mag = sl.Timestamp()
    
    '''
    Check if the new timestamp is higher than the reference one,
    and if yes, save the current as reference
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
Base Zed Model to instantiate
'''
class ZedModel():
    def __init__(self):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.NONE
        self.info = self.zed.get_camera_information()
        self.cam_model = self.info.camera_model
        self.ts_handler = TimestampHandler()
        self.logger = logging.getLogger()
    
    def openCamera(self):
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.logger.warning(repr(err))
            self.zed.close()
            exit(1)
        return True

    def get_camera_configuration(self):
        self.logger.info(self.info)
        self.logger.info("Camera Model: " + str(self.cam_model))
        self.logger.info("Serial Number: " + str(self.info.serial_number))
        self.logger.info("Camera Firmware: " + str(self.info.camera_configuration.firmware_version))
        self.logger.info("Sensors Firmware: " + str(self.info.sensors_configuration.firmware_version))

        return self.info

    def get_sensor_configuration(self):
        sensors = ['accelerometer', 'gyroscope', 'magnetometer', 'barometer']

        for sensor in sensors:
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
    
    def get_sensor_data(self):
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
                # Linear acceleration
                linear_acceleration = self.sensors_data.get_imu_data().get_linear_acceleration()
                self.logger.info(" \t Acceleration: [ {0} {1} {2} ] [m/sec^2]".format(linear_acceleration[0], 
                                                                                      linear_acceleration[1], 
                                                                                      linear_acceleration[2]))
                # Angular velocities
                angular_velocity = self.sensors_data.get_imu_data().get_angular_velocity()  
                self.logger.info(" \t Angular Velocities: [ {0} {1} {2} ] [deg/sec]".format(angular_velocity[0], 
                                                                                            angular_velocity[1], 
                                                                                            angular_velocity[2]))
                if self.timestamp_handler.is_new(self.sensors_data.get_magnetometer_data()):
                    # Magnetic field
                    magnetic_field_calibrated = self.sensors_data.get_magnetometer_data().get_calibrated_magnetic_field()
                    self.logger.info(" \t Magnetic Field: [ {0} {1} {2} ] [uT]".format(magnetic_field_calibrated[0], 
                                                                                       magnetic_field_calibrated[1], 
                                                                                       magnetic_field_calibrated[2]))
                if self.timestamp_handler.is_new(self.sensors_data.get_barometer_data()):
                    # Atmospheric pressure
                    pressure = self.sensors_data.get_barometer_data().pressure()
                    self.logger.info(" \t Pressure: {0} [hPa]".format(pressure))

        if self.timestamp_handler.is_new(self.sensors_data.get_imu_data()) \
        and self.timestamp_handler.is_new(self.sensors_data.get_magnetometer_data()) \
        and self.timestamp_handler.is_new(self.sensors_data.get_barometer_data()):
            return self.sensors_data
        