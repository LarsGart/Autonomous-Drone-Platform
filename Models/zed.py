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
from models.logger import Logger


class Zed(Logger):
    '''
    Initializes the Zed camera object
    '''
    def __init__(self):
        
        # Create a ZEDCamera object
        self.camera = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode (default fps: 60)

        # Use a right-handed Y-up coordinate system
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.coordinate_units = sl.UNIT.METER  # Set units in meters
        logs = []
        logs.append(f"ZedModel Created: {self.camera}")

        self.imu_data = sl.IMUData()

        self.close()

        # Open the camera
        err = self.camera.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            logs.append(f"{err}")
            self.camera.close()
            # exit(1)

        # Enable positional tracking with default parameters
        py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
        tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
        err = self.camera.enable_positional_tracking(tracking_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            logs.append(f"ZedModel: {err}")
            self.camera.close()
            exit(1)

        # Enable spatial mapping
        # mapping_parameters = sl.SpatialMappingParameters(map_type=sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD)
        # err = self.camera.enable_spatial_mapping(mapping_parameters)
        # if err != sl.ERROR_CODE.SUCCESS:
        #     self.logger.warning(f"ZedModel: {err}")
        #     self.camera.close()
        #     exit(1)

        # Define camera information
        self.info = self.camera.get_camera_information()
        self.sensors = ['accelerometer', 'gyroscope']

        self.camera_pose = sl.Pose()

        # Initialize previous position to (0, 0, 0)
        self.camera.get_position(self.camera_pose, sl.REFERENCE_FRAME.WORLD)
        self.previous_position = self.camera_pose.get_translation(sl.Translation()).get()
        super().__init__()
        for log in logs:
            self.log(log)


    def close(self):
        if self.camera.is_opened():
            self.camera.disable_spatial_mapping()
            self.camera.close()
            self.log("Camera closed")

    def get_y_angular_velocity(self):
        self.sensors_data = sl.SensorsData()
        if self.camera.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            angular_velocity = self.sensors_data.get_imu_data().get_angular_velocity()
            return angular_velocity[1]

    def get_quaternion(self):
        self.sensors_data = sl.SensorsData()
        if self.camera.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            quaternion = self.sensors_data.get_imu_data().get_pose().get_orientation().get()
            return quaternion
        else:
            self.log('IMU data has not been updated.', level='WARNING')
            return [0, 0, 0, 0]
    
    '''Returns the euler angles in radians: in the order of roll, pitch, yaw'''
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
     
        return roll, pitch, yaw

    def get_euler_in_degrees(self):
        roll, pitch, yaw = self.get_euler()
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    '''
    Captures a single frame and calculates the difference in position
    from the previous timestep. It returns the position difference as a 3D vector.
    '''
    def get_position_diff(self):
        # Grab data for one frame
        if self.camera.grab() == sl.ERROR_CODE.SUCCESS:
            current_position = self.get_pos_global()
            position_diff = current_position - self.previous_position  # Calculate the difference in position from the previous timestep
            self.previous_position = current_position  # Update the previous position to the current position for the next iteration
            return position_diff
        return [0, 0, 0]
    

    def get_pos_global(self):
        if self.camera.grab() == sl.ERROR_CODE.SUCCESS:
            # Get the current pose information
            self.camera.get_position(self.camera_pose, sl.REFERENCE_FRAME.WORLD)
            # Get current zed position
            curr_position = self.camera_pose.get_translation(sl.Translation()).get()
            return curr_position
        return [0, 0, 0]
        

    def get_pos_relative(self):
        if self.camera.grab() == sl.ERROR_CODE.SUCCESS:
            # Get the current pose information
            self.camera.get_position(self.camera_pose, sl.REFERENCE_FRAME.CAMERA)
            # Get current zed position
            curr_position = self.camera_pose.get_translation(sl.Translation()).get()
            return curr_position
        return [0, 0, 0]
        