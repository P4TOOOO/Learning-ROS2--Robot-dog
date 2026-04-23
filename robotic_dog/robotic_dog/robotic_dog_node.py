import numpy as np
import math as m
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg


# ═══════════════════════════════════════════════════════════════════════
#  LEG MAPING
#
#    hip1: x= 0.073, y= 0.172  → Front-left  (leg 0)
#    hip4: x= 0.073, y=-0.031  → Front-right   (leg 1)
#    hip2: x=-0.125, y= 0.172  → back-left    (leg 2)
#    hip3: x=-0.125, y=-0.031  → back-right     (leg 3)
# ═══════════════════════════════════════════════════════════════════════
LEG_JOINTS = {
    0: ['hip1', 'thigh1', 'shin1'],   # (FL)
    1: ['hip4', 'thigh4', 'shin4'],   # (FR)
    2: ['hip2', 'thigh2', 'shin2'],   # (BL)
    3: ['hip3', 'thigh3', 'shin3'],   # (BR)
}

JOINT_NAMES = (
    LEG_JOINTS[0] + LEG_JOINTS[1] +
    LEG_JOINTS[2] + LEG_JOINTS[3]
)

# ═══════════════════════════════════════════════════════════════════════
#  LEGS SIGNS -- came from URDF
# ═══════════════════════════════════════════════════════════════════════
THIGH_SIGN = {0: -1, 1:  1, 2: -1, 3:  1}
SHIN_SIGN  = {0:  1, 1: -1, 2:  1, 3: -1}

# ═══════════════════════════════════════════════════════════════════════
#  ATTITUDE CORRECTION FACTORS BY LEG
#
#  PITCH (front falling = pitch > 0):
#    front legs need to push more → negative factor 
#    back legs don't need to push → positive factor 
#
#  ROLL (right side falling = roll > 0):
#    right legs need to push more → fator positivo
#    left legs don't need to push → fator negativo
#
#         FL   FR   BL   BR
# ═══════════════════════════════════════════════════════════════════════
PITCH_FACTOR = {0: -1.0, 1: -1.0, 2:  1.0, 3:  1.0}
ROLL_FACTOR  = {0: -1.0, 1:  1.0, 2: -1.0, 3:  1.0}

# ═══════════════════════════════════════════════════════════════════════
#  HOME POSITION
 # ═══════════════════════════════════════════════════════════════════════
HIP_HOME   =  0.0
THIGH_HOME =  0.6
SHIN_HOME  =  1.2

# ═══════════════════════════════════════════════════════════════════════
#  TROT PARAMETERS
# ═══════════════════════════════════════════════════════════════════════
#   FL(0)+BR(3),  FR(1)+BL(2)
TROT_OFFSET = {0: 0.0, 1: 0.5, 2: 0.5, 3: 0.0}

A_THIGH    = 0.3
A_SHIN     = 0.25
DUTY       = 0.4   # Swing cicle 
CYCLE_TIME = 1.5    # seconds per cicle

# ═══════════════════════════════════════════════════════════════════════
#  CoM velocity control (PI)
# ═══════════════════════════════════════════════════════════════════════
TARGET_LINEAR_X  = 0.5
TARGET_ANGULAR_Z = 0.0

Kp_linear  = 0.8
Ki_linear  = 0.05
Kp_angular = 0.2
Ki_angular = 0.01

MAX_THIGH_AMPLITUDE = 0.3
MIN_THIGH_AMPLITUDE = 0.05
MAX_YAW_BIAS        = 0.5

# ═══════════════════════════════════════════════════════════════════════
#  ATTITUDE CONTROL (PD)
# ═══════════════════════════════════════════════════════════════════════
Kp_pitch = 0.8       
Kd_pitch = 0.05      
Kp_roll  = 0.2       
Kd_roll  = 0.01      

MAX_PITCH_OFFSET = 0.40   # pitch limit correction (rad)
MAX_ROLL_OFFSET  = 0.15   # rad

# soften the attitude correction
ATTITUDE_ALPHA = 0.15     # 0=much soften, 1=no filter


class RoboticDogNode(Node):

    def __init__(self):
        super().__init__("robotic_dog_node")

        # ── Odom ────────────────────────────────────────────────
        self.current_linear_x  = 0.0
        self.current_angular_z = 0.0

        # ── Velocity PI ─────────────────────────────────────────
        self.integral_linear  = 0.0
        self.integral_angular = 0.0

        # ── Adaptative parameters ───────────────────────────────────
        self.adaptive_A_THIGH = A_THIGH
        self.yaw_bias         = 0.0

        # ── Trot ───────────────────────────────────────────────────
        self.cycle_time = CYCLE_TIME
        self.gait_phase = 0.0
        self.last_time  = self.get_clock().now()

        # ── TF and odom ───────────────────────────────────────────
        self.tf_buffer   = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_pos    = np.zeros(3)
        self.odom_rot    = np.eye(3)

        # ── Velocity control filters ─────────────────────
        self.filtered_correction_linear  = 0.0
        self.filtered_correction_angular = 0.0
        self.alpha                        = 0.2
        self.max_delta_A_thigh_per_s      = 0.3
        self.max_delta_yaw_bias_per_s     = 0.2
        self.last_control_time            = self.get_clock().now()

        # ── Cache per leg ──────────────────────────────────────────
        self.cached_A_thigh  = [A_THIGH] * 4
        self.cached_yaw_bias = [0.0]     * 4

        # ── IMU / attitude ────────────────────────────────────────────
        self.current_pitch = 0.0
        self.current_roll  = 0.0

        # Previous errors for the derivative term
        self.last_pitch_error = 0.0
        self.last_roll_error  = 0.0

        # Filtered corrections (PD output after low-pass filter)
        self.filtered_pitch_corr = 0.0
        self.filtered_roll_corr  = 0.0

        self.last_imu_time  = self.get_clock().now()
        self.imu_received   = False

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom',      self.odom_callback, 10)
        self.create_subscription(Imu,      '/imu',       self.imu_callback,  10)

        # ── Publisher ────────────────────────────────────────────────
        self.pub = self.create_publisher(
            JointTrajectory,
            '/position_controllers/joint_trajectory',
            10
        )

        # ── Timers ───────────────────────────────────────────────────
        self.home_published = False
        self.create_timer(0.5,  self.publish_home)
        self.create_timer(0.02, self.movement_callback)   # 50 Hz
        self.create_timer(0.1,  self.control_callback)    # 10 Hz

        self.get_logger().info(
            "RoboticDogNode iniciado — controle de atitude ativo.\n"
            f"  Kp_pitch={Kp_pitch}  Kd_pitch={Kd_pitch}\n"
            f"  Kp_roll={Kp_roll}    Kd_roll={Kd_roll}"
        )

    # ─────────────────────────────────────────────────────────────────
    #  SENSORS CALLBACK
    # ─────────────────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_pos = np.array([p.x, p.y, p.z])
        x, y, z, w = q.x, q.y, q.z, q.w
        self.odom_rot = np.array([
            [1-2*(y*y+z*z),  2*(x*y-w*z),  2*(x*z+w*y)],
            [  2*(x*y+w*z),1-2*(x*x+z*z),  2*(y*z-w*x)],
            [  2*(x*z-w*y),  2*(y*z+w*x),1-2*(x*x+y*y)],
        ])
        self.current_linear_x  = msg.twist.twist.linear.x
        self.current_angular_z = msg.twist.twist.angular.z

    def imu_callback(self, msg: Imu):
        q = msg.orientation

        # Quaternion → pitch (Y) e roll (X)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))   # clamp para evitar NaN
        pitch = m.asin(sinp)

        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x*q.x + q.y*q.y)
        roll = m.atan2(sinr, cosr)

        self.current_pitch = pitch
        self.current_roll  = roll
        self.last_imu_time = self.get_clock().now()

        if not self.imu_received:
            self.get_logger().info(
                f"IMU conectada! pitch={pitch:.3f} rad  roll={roll:.3f} rad"
            )
            self.imu_received = True

    # ─────────────────────────────────────────────────────────────────
    #  HOME POS 
    # ─────────────────────────────────────────────────────────────────
    def publish_home(self):
        if self.home_published:
            return
        positions = []
        for leg in range(4):
            thigh = THIGH_SIGN[leg] * THIGH_HOME
            shin  = SHIN_SIGN[leg]  * SHIN_HOME
            positions += [HIP_HOME, thigh, shin]
        self._send_joints(positions, duration_ms=1500)
        self.home_published = True
        self.get_logger().info("Posição HOME publicada.")

    # ─────────────────────────────────────────────────────────────────
    #  VELOCITY CONTROL — 10 Hz
    # ─────────────────────────────────────────────────────────────────
    def control_callback(self):
        if not self.home_published:
            return

        now = self.get_clock().now()
        dt  = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now
        if dt <= 0:
            return

        err_lin = TARGET_LINEAR_X  - self.current_linear_x
        err_ang = TARGET_ANGULAR_Z - self.current_angular_z

        self.integral_linear  = max(min(self.integral_linear  + err_lin * dt,  1.0), -1.0)
        self.integral_angular = max(min(self.integral_angular + err_ang * dt,  0.5), -0.5)

        raw_lin = Kp_linear  * err_lin + Ki_linear  * self.integral_linear
        raw_ang = Kp_angular * err_ang + Ki_angular * self.integral_angular

        self.filtered_correction_linear  = (self.alpha * raw_lin +
                                            (1 - self.alpha) * self.filtered_correction_linear)
        self.filtered_correction_angular = (self.alpha * raw_ang +
                                            (1 - self.alpha) * self.filtered_correction_angular)

        # Thigh amplitude (controls linear velocity)
        max_dA  = self.max_delta_A_thigh_per_s * dt
        target_A = A_THIGH + self.filtered_correction_linear
        delta_A  = max(-max_dA, min(target_A - self.adaptive_A_THIGH, max_dA))
        self.adaptive_A_THIGH = max(MIN_THIGH_AMPLITUDE,
                                    min(self.adaptive_A_THIGH + delta_A, MAX_THIGH_AMPLITUDE))

        # Yaw bias (Control angular velocity)
        max_dy   = self.max_delta_yaw_bias_per_s * dt
        delta_yaw = max(-max_dy, min(self.filtered_correction_angular - self.yaw_bias, max_dy))
        self.yaw_bias = max(-MAX_YAW_BIAS, min(self.yaw_bias + delta_yaw, MAX_YAW_BIAS))

    # ─────────────────────────────────────────────────────────────────
    #  ENGINEER WITH ATTITUDE CONTROL — 50 Hz
    # ─────────────────────────────────────────────────────────────────
    def movement_callback(self):
        if not self.home_published:
            return

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0:
            return

        self.gait_phase = (self.gait_phase + dt / self.cycle_time) % 1.0

        # ── Calculate PD corrections ──────────────────────────
        pitch_corr, roll_corr = self._compute_attitude_correction(now)

        # ── Generate positions to each leg ────────────────────────────
        positions = []
        for leg in range(4):
            # Phase offset for yaw (right legs vs left legs)
            phase_offset = self.yaw_bias if leg in (1, 3) else -self.yaw_bias
            phase = (self.gait_phase + TROT_OFFSET[leg] + phase_offset) % 1.0

            # Freezes parameters at the start of the swing for smoothness.
            if phase < 0.05:
                self.cached_A_thigh[leg]  = self.adaptive_A_THIGH
                self.cached_yaw_bias[leg] = self.yaw_bias

            A_thigh_leg = self.cached_A_thigh[leg]
            ts = THIGH_SIGN[leg]
            ss = SHIN_SIGN[leg]

            # ── Attitude Offset for this Leg ──────────────────────
            # Applied in BOTH phases (swing and stance) to avoid
            # discontinuities in the transition
            leg_offset = (PITCH_FACTOR[leg] * pitch_corr +
                          ROLL_FACTOR[leg]  * roll_corr)

            # The adjusted home position incorporates attitude correction.
            thigh_home_adj = THIGH_HOME + leg_offset
            shin_home_adj  = SHIN_HOME  + leg_offset * 0.5  # Shin with half the effect.

            # ── Kinematics of trotting ────────────────────────────────────
            if phase < DUTY:
                # Swing: leg in the air, sinusoidal trajectory
                s = phase / DUTY
                thigh = ts * thigh_home_adj - ts * A_thigh_leg * m.sin(m.pi * s)
                shin  = ss * shin_home_adj  + ss * A_SHIN      * m.sin(m.pi * s)
            else:
                # Stance: leg on the ground, smooth linear retreat.
                s = (phase - DUTY) / (1.0 - DUTY)
                thigh = ts * thigh_home_adj + ts * A_thigh_leg * (2*s - 1) * 0.5
                shin  = ss * shin_home_adj  - ss * A_SHIN * 0.15

            positions += [HIP_HOME, thigh, shin]

        self._send_joints(positions, duration_ms=20)

    # ─────────────────────────────────────────────────────────────────
    #  Calculation of the attitude PD (with low-pass filter)
    # ─────────────────────────────────────────────────────────────────
    def _compute_attitude_correction(self, now):
        """
        Retorna (pitch_corr, roll_corr) em radianos, já filtrados e limitados.

        Convenção:
          - pitch > 0 → nariz caindo → pernas dianteiras devem empurrar mais
          - roll  > 0 → lado direito caindo → pernas direitas devem empurrar mais
          O sinal correto é cuidado pelos PITCH_FACTOR / ROLL_FACTOR por perna.
        """
        # Error signal: we want pitch=0 and roll=0
        error_pitch = -self.current_pitch
        error_roll  = -self.current_roll

        # Derivative term (derived from the error)
        dt_imu = (now - self.last_imu_time).nanoseconds * 1e-9
        if 0.0 < dt_imu < 0.5:
            d_pitch = (error_pitch - self.last_pitch_error) / dt_imu
            d_roll  = (error_roll  - self.last_roll_error)  / dt_imu
        else:
            d_pitch = d_roll = 0.0

        self.last_pitch_error = error_pitch
        self.last_roll_error  = error_roll

        # PD 
        raw_pitch = Kp_pitch * error_pitch + Kd_pitch * d_pitch
        raw_roll  = Kp_roll  * error_roll  + Kd_roll  * d_roll

        # Low-pass filter to smooth the correction.
        self.filtered_pitch_corr = (ATTITUDE_ALPHA * raw_pitch +
                                    (1 - ATTITUDE_ALPHA) * self.filtered_pitch_corr)
        self.filtered_roll_corr  = (ATTITUDE_ALPHA * raw_roll  +
                                    (1 - ATTITUDE_ALPHA) * self.filtered_roll_corr)

        # Saturation (prevents excessively sudden movements)
        pitch_corr = max(-MAX_PITCH_OFFSET, min(self.filtered_pitch_corr, MAX_PITCH_OFFSET))
        roll_corr  = max(-MAX_ROLL_OFFSET,  min(self.filtered_roll_corr,  MAX_ROLL_OFFSET))

        # Diagnostic log (1x per second)
        self.get_logger().info(
            f"ATT | pitch={self.current_pitch:+.3f} err={error_pitch:+.3f} corr={pitch_corr:+.3f} | "
            f"roll={self.current_roll:+.3f} err={error_roll:+.3f} corr={roll_corr:+.3f}",
            throttle_duration_sec=1.0
        )

        return pitch_corr, roll_corr

    # ─────────────────────────────────────────────────────────────────
    #  PUBLISHING LEG COMANDS
    # ─────────────────────────────────────────────────────────────────
    def _send_joints(self, positions, duration_ms: int):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions        = positions
        pt.velocities       = [0.0] * 12
        pt.time_from_start  = DurationMsg(sec=0, nanosec=duration_ms * 1_000_000)
        msg.points = [pt]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoboticDogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
