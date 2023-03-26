'''
Author: Lars Gartenberg
Editor: Jerin Abraham

The ZedModel class has methods to:
- open and close the ZED mini camera
- get camera and sensor configuration information
- retrieve accelerometer, gyroscope data
- quaternion, and euler angle values
- get the position delta
'''
import pyzed.sl as sl
import math
import logging
import math
from datetime import datetime


class ZedModel:
    '''
    Initializes the Zed camera object
    '''
    def __init__(self):
        logging.basicConfig(filename=f"../Logs/{self.__class__.__name__}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
                            ,level=logging.DEBUG
                            ,format='%(asctime)s:%(levelname)s:%(message)s')
        self.logger = logging.getLogger()
        
        # Create a ZEDCamera object
        self.zed = sl.Camera()
        self.logger.info(f"ZedModel: {self.zed}")

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode (default fps: 60)
        self.logger.info(f"ZedModel: {init_params}")

        # Use a right-handed Y-up coordinate system
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.coordinate_units = sl.UNIT.METER  # Set units in meters
        self.logger.info(f"ZedModel: {init_params}")

        self.closeCamera()

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.logger.error(f"ZedModel: {err}")
            self.zed.close()
            exit(1)

        # Enable positional tracking with default parameters
        py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
        tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
        err = self.zed.enable_positional_tracking(tracking_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            self.logger.warning(f"ZedModel: {err}")
            self.zed.close()
            exit(1)

        # Enable spatial mapping
        mapping_parameters = sl.SpatialMappingParameters(map_type=sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD)
        err = self.zed.enable_spatial_mapping(mapping_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            self.logger.warning(f"ZedModel: {err}")
            self.zed.close()
            exit(1)

        # Define camera information
        self.info = self.zed.get_camera_information()
        self.sensors = ['accelerometer', 'gyroscope']
        self.logger.info("ZedSensorModel initialized")

        self.zed_pose = sl.Pose()

        # Initialize previous position to (0, 0, 0)
        self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
        self.prev_position = self.zed_pose.get_translation(sl.Translation()).get()


    def closeCamera(self):
        if self.zed.is_opened():
        #    Disable spatial mapping and close the camera
            self.zed.disable_spatial_mapping()
            self.zed.close()
            self.logger.info("Camera closed")
        else:
            self.logger.info("Camera not open")


    def get_camera_configuration(self):
        self.logger.info(self.info)
        self.logger.info(f"Camera Model: {self.info.camera_model}")
        self.logger.info(f"Serial Number: {self.info.serial_number}")
        self.logger.info(f"Camera Firmware: {self.info.camera_configuration.firmware_version}")
        self.logger.info(f"Sensors Firmware: {self.info.sensors_configuration.firmware_version}")
        return self.info
    

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

            
    def get_sensor_data(self, sensor_type=None):
        if not self.zed.is_opened():
            self.logger.warning("Sensor data not available because the ZED is not opened")
            return None

        self.sensors_data = sl.SensorsData()
        if self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) != sl.ERROR_CODE.SUCCESS:
            return None

        if sensor_type == 'all':
            return {sensor: self.get_sensor_data(sensor) for sensor in self.sensors}
        elif sensor_type == 'accelerometer':
            linear_acceleration = self.sensors_data.get_imu_data().get_linear_acceleration()
            self.logger.info(f"\tAcceleration: {linear_acceleration} [m/sec^2]")
            return linear_acceleration
        elif sensor_type == 'gyroscope':
            angular_velocity = self.sensors_data.get_imu_data().get_angular_velocity()
            self.logger.info(f"\tAngular Velocities: {angular_velocity} [deg/sec]")
            return angular_velocity
        else:
            self.logger.warning("Sensor type not available")
            return None
        

    def get_quaternion(self):
        self.sensors_data = sl.SensorsData()
        if self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            quaternion = self.sensors_data.get_imu_data().get_pose().get_orientation().get()
            self.logger.info(" \t Orientation: [ Ox: {0}, Oy: {1}, Oz {2}, Ow: {3} ]".format(*quaternion))
            return quaternion
        else:
            self.logger.warning("IMU data has not been updated")
            return None
    

    def get_euler(self):
        q = self.get_quaternion()
        x, y, z, w = q[0], q[1], q[2], q[3]

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return [roll_x, pitch_y, yaw_z]
    

    '''
    Captures a single frame and calculates the difference in position
    from the previous timestep. It returns the position difference as a 3D vector.
    '''
    def get_position_diff(self):
        # Grab data for one frame
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Get the current position of the camera
            self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)

            # Get current zed position
            curr_position = self.zed_pose.get_translation(sl.Translation()).get()

            # Calculate the difference in position from the previous timestep
            position_diff = curr_position - self.prev_position

            # Update the previous position to the current position for the next iteration
            self.prev_position = curr_position
            self.logger.info(f"Position diff: {position_diff} [m]")
            print((f"Position diff: {position_diff} [m]"))
            return position_diff
    