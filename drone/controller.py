import numpy as np
from lqr import LQR
from xiao import Xiao

"""
------------- Channel Mappings -------------

   CH1 (ROLL):     LEFT = 1000, RIGHT = 2000
   CH2 (PITCH):    DOWN = 1000, UP    = 2000
   CH3 (THROTTLE): DOWN = 1000, UP    = 2000
   CH4 (YAW):      LEFT = 1000, RIGHT = 2000

-------------- Motor Mappings --------------

                 3  ^  0
                  \_|_/
                  |   |
                  |___|
                  /   \
                 2     1
"""

class Controller():
    '''Flight controller using LQR for attitude (roll, pitch, yaw).

      - Uses Zed._get_state_space_representation() to read angles and angular velocities.
      - Linearized small-angle model: angle_ddot = torque / I_axis.
      - State for controller: [angle, ang_vel] per axis stacked -> 6 states.
      - Input: torques for roll, pitch, yaw (3 inputs).
      - Motor mapping is the same as in your old Controller (4 motors).
      - All units: angles in radians, angular velocities in rad/s
      - Torque -> motor speed conversions require tuning.
    '''

    THROTTLE_CUTOFF = 1012
    MAX_MOTOR = 2000
    MIN_MOTOR = 1000

    def __init__(self, zed) -> None:
        self.zed = zed
        self.xiao = Xiao()

        self.dt = 1.0 / 50.0  # controller sample time (s)
        self.inertia = 1.0 / 50.0  # controller sample time (s)

        # build discrete A/B for attitude (3 axes, each a 2-state double integrator)
        A_c = np.array([[0.0, 1.0], [0.0, 0.0]])  # continuous-time per-axis
        # B_c = [[0],[1/I]]
        # build block-diagonal for 3 axes
        Ac = np.kron(np.eye(3), A_c)  # 6x6

        # B_blocks = []
        # for I_axis in self.inertia:
        #     B_blocks.append(np.array([[0.0], [1.0 / I_axis]]))
        B_blocks = np.array([[0.0], [0.0], [1.0 / self.inertia]]) # patch
        Bc = np.zeros((6, 3))
        for i in range(3):
            Bc[2 * i:2 * i + 2, i:i + 1] = B_blocks[i]

        # discretize (simple Euler / exact via matrix exponential could be used; use first-order Euler for simplicity)
        # x_{k+1} = (I + A_c*dt) x_k + B_c * dt u_k  (forward Euler)
        self.A = np.eye(6) + Ac * self.dt
        self.B = Bc * self.dt

        # LQR design: tune Q and R
        # Penalize angle error relatively more than angular rate; R penalizes torque magnitude
        q_angle = 10.0
        q_rate = 1.0
        Q_axis = np.diag([q_angle, q_rate])
        Q = np.kron(np.eye(3), Q_axis)  # 6x6

        R = np.diag([0.1, 0.1, 0.1])  # penalize actuator effort

        # compute K
        K, _ = LQR._solve_discrete_lqr(self.A, self.B, Q, R)  # K shape (3,6)
        self.K = K

        # motor mixing scale: maps torque (Nm) to motor speed change (units consistent with your motors)
        # This is a rough conversion factor that must be calibrated for your platform.
        self.thrust_to_speed = 200.0  # placeholder: how many "speed units" per Nm

        # keep local state for debugging/telemetry
        self.last_u = np.zeros(3)
        self.last_x = np.zeros(6)

    @staticmethod
    def _clamp_motor_speeds(speeds: np.ndarray) -> np.ndarray:
        speeds = np.round(speeds).astype(int)
        speeds = np.clip(speeds, Controller.MIN_MOTOR, Controller.MAX_MOTOR)
        return speeds

    def _read_attitude_state(self) -> np.ndarray:
        '''Read attitude-related state from ZED and produce state vector x = [roll, roll_rate, pitch, pitch_rate, yaw, yaw_rate]'''
        ssr = self.zed._get_state_space_representation()
        # angles (radians) and angular velocities (rad/s)
        roll = ssr.roll
        pitch = ssr.pitch
        yaw = ssr.yaw
        wx = ssr.wx
        wy = ssr.wy
        wz = ssr.wz
        # stack as [roll, roll_rate, pitch, pitch_rate, yaw, yaw_rate]
        # note: sign conventions matter — align with your PID implementation if switching
        x = np.array([roll, wx, pitch, wy, yaw, wz], dtype=float)
        return x

    def _mix_torques_to_motor_delta(self, torques: np.ndarray) -> np.ndarray:
        '''
        Convert axis torques [tau_roll, tau_pitch, tau_yaw] to per-motor delta speed values.
        Using the same mixing as original code:
        motors = [
          throttle + pitch + roll + yaw,
          throttle - pitch + roll - yaw,
          throttle - pitch - roll + yaw,
          throttle + pitch - roll - yaw
        ]
        We treat roll/pitch/yaw contributions as "speed offsets" via thrust_to_speed scaling.
        '''
        roll_delta = torques[0] * self.thrust_to_speed
        pitch_delta = torques[1] * self.thrust_to_speed
        yaw_delta = torques[2] * self.thrust_to_speed

        m0 =  pitch_delta + roll_delta + yaw_delta
        m1 = -pitch_delta + roll_delta - yaw_delta
        m2 = -pitch_delta - roll_delta + yaw_delta
        m3 =  pitch_delta - roll_delta - yaw_delta
        return np.array([m0, m1, m2, m3], dtype=float)

    def _update(self, throttle_pwm: int, stick_roll: float, stick_pitch: float, stick_yaw: float) -> None:
        '''
        Single controller step.

        Args:
            throttle_pwm: raw throttle PWM from RX (1000-2000)
            stick_roll: stick position normalized [-1,1] (left->right)
            stick_pitch: stick position normalized [-1,1] (down->up)
            stick_yaw: stick position normalized [-1,1] (left->right)
        '''
        # failsafe on low throttle
        if throttle_pwm <= self.THROTTLE_CUTOFF:
            # zero throttle / motors stop
            self.motors.zero_throttle()
            return

        # Build setpoint angles (radians). Previously used 10 degrees for max stick deflection.
        max_angle_rad = np.deg2rad(10.0)
        roll_sp = max_angle_rad * stick_roll
        pitch_sp = max_angle_rad * stick_pitch
        yaw_sp = max_angle_rad * stick_yaw  # small yaw setpoint

        # State and error
        x = self._read_attitude_state()  # [roll, roll_rate, pitch, pitch_rate, yaw, yaw_rate]
        x_ref = np.array([roll_sp, 0.0, pitch_sp, 0.0, yaw_sp, 0.0], dtype=float)
        e = x - x_ref

        # LQR control: u = -K e  (u are torques)
        u = -self.K @ e  # shape (3,)
        self.last_u = u.copy()
        self.last_x = x.copy()

        # mix torques into motor delta speeds
        motor_deltas = self._mix_torques_to_motor_delta(u)

        # throttle baseline mapping: map PWM [1000,2000] -> motor baseline [MIN_MOTOR..MAX_MOTOR] or a percent
        # Use simple linear mapping: baseline = throttle_pwm (assume motors expect same PWM units)
        baseline = throttle_pwm

        motor_speeds = baseline + motor_deltas
        motor_speeds = self._clamp_motor_speeds(motor_speeds)
        self.xiao._set_speeds(list(motor_speeds))
