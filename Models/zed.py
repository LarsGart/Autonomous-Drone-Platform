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


class Zed:
    '''
    Initializes the Zed camera object
    '''
    def __init__(self,log=False):
        self.log = log
        if self.log:
            logging.basicConfig(filename=f"../Logs/{self.__class__.__name__}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
                                ,level=logging.DEBUG
                                ,format='%(asctime)s:%(levelname)s:%(message)s')
            self.logger = logging.getLogger()
        
        # Create a ZEDCamera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode (default fps: 60)

        # Use a right-handed Y-up coordinate system
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.coordinate_units = sl.UNIT.METER  # Set units in meters

        if self.log:
            self.logger.info(f"ZedModel Created: {self.zed}")

        self.imu_data = sl.IMUData()

        self.close()

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            if self.log:
                self.logger.error(f"ZedModel: {err}")
            self.zed.close()
            exit(1)

        # Enable positional tracking with default parameters
        py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
        tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
        err = self.zed.enable_positional_tracking(tracking_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            if log:
                self.logger.warning(f"ZedModel: {err}")
            self.zed.close()
            exit(1)
        if self.log:
            self.logger.info("Successfully initialized ZED")

        # Enable spatial mapping
        # mapping_parameters = sl.SpatialMappingParameters(map_type=sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD)
        # err = self.zed.enable_spatial_mapping(mapping_parameters)
        # if err != sl.ERROR_CODE.SUCCESS:
        #     self.logger.warning(f"ZedModel: {err}")
        #     self.zed.close()
        #     exit(1)

        # Define camera information
        self.info = self.zed.get_camera_information()
        self.sensors = ['accelerometer', 'gyroscope']

        self.zed_pose = sl.Pose()

        # Initialize previous position to (0, 0, 0)
        self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
        self.previous_position = self.zed_pose.get_translation(sl.Translation()).get()


    def close(self):
        if self.zed.is_opened():
        # Disable spatial mapping and close the camera
            self.zed.disable_spatial_mapping()
            self.zed.close()
            if self.log:
                self.logger.info("Camera closed")


    def get_camera_configuration(self):
        if self.log:
            self.logger.info(self.info)
            self.logger.info(f"Camera Model: {self.info.camera_model}")
            self.logger.info(f"Serial Number: {self.info.serial_number}")
            self.logger.info(f"Camera Firmware: {self.info.camera_configuration.firmware_version}")
            self.logger.info(f"Sensors Firmware: {self.info.sensors_configuration.firmware_version}")
        return self.info

    def get_y_angular_velocity(self):
        self.sensors_data = sl.SensorsData()
        if self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            angular_velocity = self.sensors_data.get_imu_data().get_angular_velocity()
            return angular_velocity[1]

    def get_quaternion(self):
        self.sensors_data = sl.SensorsData()
        if self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            quaternion = self.sensors_data.get_imu_data().get_pose().get_orientation().get()
            return quaternion
        else:
            if self.log:
                self.logger.warning("IMU data has not been updated")
            return None
    

    def get_euler(self):
        q = self.get_quaternion()
        x, y, z, w = q[0], q[1], q[2], q[3]

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        pitch = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        yaw = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        roll = math.atan2(t3, t4)
     
        return {'roll': roll, 'pitch': pitch, 'yaw': yaw}

    def get_euler_in_degrees(self):
        return {key: value * 180 / math.pi for key, value in self.get_euler().items()}  
    

    '''
    Captures a single frame and calculates the difference in position
    from the previous timestep. It returns the position difference as a 3D vector.
    '''
    def get_position_diff(self):
        # Grab data for one frame
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            current_position = self.get_pos_global()
            position_diff = current_position - self.previous_position  # Calculate the difference in position from the previous timestep
            self.previous_position = current_position  # Update the previous position to the current position for the next iteration
            return position_diff
    

    def get_pos_global(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Get the current pose information
            self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)

            # Get current zed position
            curr_position = self.zed_pose.get_translation(sl.Translation()).get()

            return curr_position
        

    def get_pos_relative(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Get the current pose information
            self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.CAMERA)

            # Get current zed position
            curr_position = self.zed_pose.get_translation(sl.Translation()).get()

            return curr_position
        