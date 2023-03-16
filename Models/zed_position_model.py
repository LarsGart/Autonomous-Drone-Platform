'''
Author: Lars Gartenberg

The ZedPositionModel class has methods to:
- open and close the ZED mini camera
- get camera and sensor configuration information
- retrieve accelerometer, gyroscope data
- quaternion, and euler angle values
'''
import pyzed.sl as sl
import logging
from datetime import datetime


class ZedPositionModel:
    def __init__(self):
        # Create a ZEDCamera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode (default fps: 60)

        # Use a right-handed Y-up coordinate system
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.coordinate_units = sl.UNIT.METER  # Set units in meters

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        # Enable positional tracking with default parameters.
        py_transform = sl.Transform()
        tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
        err = self.zed.enable_positional_tracking(tracking_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        # Enable spatial mapping
        mapping_parameters = sl.SpatialMappingParameters(map_type=sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD)
        err = self.zed.enable_spatial_mapping(mapping_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        # Initialize previous position to (0, 0, 0)
        self.prev_position = sl.Translation()

    '''
    Captures a single frame and calculates the difference in position
    from the previous timestep. It returns the position difference as a 3D vector.
    '''
    def get_position_diff(self):
        # Grab data for one frame
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Get the current position of the camera
            curr_position = sl.Translation()
            self.zed.get_position(curr_position)

            # Calculate the difference in position from the previous timestep
            position_diff = curr_position.get() - self.prev_position.get()

            # Update the previous position to the current position for the next iteration
            self.prev_position = curr_position

            return position_diff

    def __del__(self):
        # Disable spatial mapping and close the camera
        self.zed.disable_spatial_mapping()
        self.zed.close()
