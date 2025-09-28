import time
import math
from collections import namedtuple
import pyzed.sl as sl

Euler = namedtuple('Euler', ['roll', 'pitch', 'yaw'])
Quaternion = namedtuple('Quaternion', ['x', 'y', 'z', 'w'])
SSR = namedtuple('StateSpaceRepresentation', ['x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw', 'wx', 'wy', 'wz'])
LinearVelocity = namedtuple('LinearVelocity', ['vx', 'vy', 'vz'])
AngularVelocity = namedtuple('AngularVelocity', ['wx', 'wy', 'wz'])


class Zed:
    '''Interface for the ZED stereo camera using the pyzed.sl SDK.'''

    def __init__(self, tracking=True, spatial_mapping=False):
        self.zed = sl.Camera()
        self.pose = sl.Pose()
        self.sensors_data = sl.SensorsData()

        init_params = sl.InitParameters()
        init_params.coordinate_units = sl.UNIT.METER
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

        if self.zed.is_opened():
            self.zed.disable_spatial_mapping()
            self._close()

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError('Failed to open ZED camera.')

        if tracking:
            tracking_params = sl.PositionalTrackingParameters(_init_pos=sl.Transform())
            if self.zed.enable_positional_tracking(tracking_params) != sl.ERROR_CODE.SUCCESS:
                self._close()
                raise RuntimeError('Failed to enable positional tracking.')

        # if spatial_mapping:
        #     mapping_params = sl.SpatialMappingParameters(map_type=sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD)
        #     if self.zed.enable_spatial_mapping(mapping_params) != sl.ERROR_CODE.SUCCESS:
        #         self._close()
        #         raise RuntimeError('Failed to enable spatial mapping.')

        # Initialize previous position
        self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
        self.previous_position = self.pose.get_translation(sl.Translation()).get()
        self.previous_time = time.time()

    def _close(self):
        self.zed.close()

    def _get_quaternion(self) -> Quaternion:
        if self._get_sensors():
            x, y, z, w = self.sensors_data.get_imu_data().get_pose().get_orientation().get()
            return Quaternion(x, y, z, w)
        return Quaternion(0, 0, 0, 0)

    def _get_euler(self) -> Euler:
        x, y, z, w = self._get_quaternion()
        t0, t1 = 2 * (w * x + y * z), 1 - 2 * (x * x + y * y)
        pitch = math.atan2(t0, t1)

        t2 = max(-1.0, min(1.0, 2 * (w * y - z * x)))
        yaw = math.asin(t2)

        t3, t4 = 2 * (w * z + x * y), 1 - 2 * (y * y + z * z)
        roll = math.atan2(t3, t4)

        return Euler(roll, pitch, yaw)

    def _get_degrees(self) -> Euler:
        return Euler(*(angle * 180 / math.pi for angle in self._get_euler()))

    def _get_sensors(self):
        '''Fetch latest sensors data.'''
        return self.zed.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS

    def _get_state_space_representation(self) -> SSR:
        x, y, z = self._get_position_global()
        vx, vy, vz = self._get_linear_velocity()
        wx, wy, wz = self._get_angular_velocity()
        roll, pitch, yaw = self._get_euler()
        return SSR(x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz)

    def _get_linear_velocity(self):
        '''Return delta position vector since last frame.'''
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            curr_x, curr_y, curr_z = self._get_position_global()
            current_time = time.time()

            diff_x = curr_x - self.previous_position
            diff_y = curr_y - self.previous_position
            diff_z = curr_z - self.previous_position

            diff_time = current_time - self.previous_time

            self.previous_position = current_position
            self.previous_time = time.time()
            return diff_x / diff_time, diff_y / diff_time, diff_z / diff_time

    def _get_angular_velocity(self):
        '''Return angular velocity [wx, wy, wz] in deg/s.'''
        if self._get_sensors():
            return AngularVelocity(*self.sensors_data.get_imu_data().get_angular_velocity())
        return AngularVelocity(0.0, 0.0, 0.0)

    def _get_position_global(self):
        '''Return position in world frame (advances camera frame).'''
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
            return self.pose.get_translation(sl.Translation()).get()

    def _get_position_relative(self):
        '''Return position in camera frame (advances camera frame).'''
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.get_position(self.pose, sl.REFERENCE_FRAME.CAMERA)
            return self.pose.get_translation(sl.Translation()).get()
