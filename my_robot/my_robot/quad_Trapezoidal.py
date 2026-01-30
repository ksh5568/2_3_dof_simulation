import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class ThreeDOFStateSpaceController(Node):
    def __init__(self):
        super().__init__('quad_state_space_controller')

        self.pitch_pub = self.create_publisher(Float64, '/pitch_wheel/command', 10)
        self.yaw_pub = self.create_publisher(Float64, '/yaw_wheel/command', 10)
        self.roll_pub = self.create_publisher(Float64, '/roll_wheel/command', 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_sensor_data)

        self.dt = 0.05  # 20 Hz
        self.state = np.zeros((6, 1))  # [theta, theta_dot, psi, psi_dot, phi, phi_dot]
        self.input = np.zeros((2, 1))  # [Vp, Vy]

        # 시스템 행렬 (3자유도 - A, B 확장)
        self.A = np.array([
            [0, 1, 0, 0, 0, 0],
            [-2.7452, -0.2830, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, -0.2701, 0, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, -1.8, -0.3]  # 가상의 롤 복원/감쇠 계수 (예시)
        ])
        self.B = np.array([
            [0, 0],
            [37.2021, 0],
            [0, 0],
            [0, 7.461],
            [0, 0],
            [0.0, 0.0]  # 롤 제어를 위한 입력 추가 시 B 행 추가 가능
        ])
        self.C = np.array([
            [1, 0, 0, 0, 0, 0],  # pitch
            [0, 0, 1, 0, 0, 0],  # yaw
            [0, 0, 0, 0, 1, 0]   # roll
        ])

        self.pitch_offset = None
        self.yaw_offset = None
        self.roll_offset = None

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('🛩️ 3-DOF State-Space Controller (Trapezoidal) Initialized')

    def imu_callback(self, msg):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
        roll = rpy[0]
        pitch = -rpy[2]
        yaw = rpy[1]

        if self.pitch_offset is None:
            self.pitch_offset = pitch
            self.yaw_offset = yaw
            self.roll_offset = roll
            self.get_logger().info(
                f'IMU offset calibrated: pitch={pitch:.2f}, yaw={yaw:.2f}, roll={roll:.2f}')
            return

        self.measured_output = np.array([
            [pitch - self.pitch_offset],
            [yaw - self.yaw_offset],
            [roll - self.roll_offset]
        ])

    def control_loop(self):
        if not hasattr(self, 'measured_output'):
            self.get_logger().warn('Waiting for IMU...')
            return

        y_target = np.zeros((3, 1))  # 목표 각도 0°
        y_error = y_target - self.measured_output

        # 단순 비례 기반 제어 입력 (2자유도)
        self.input[0, 0] = 2.0 * y_error[0, 0]
        self.input[1, 0] = 2.0 * y_error[1, 0]
        roll_command = 2.0 * y_error[2, 0]

        # Trapezoidal integration
        f_prev = self.A @ self.state + self.B @ self.input
        x_predict = self.state + self.dt * f_prev
        f_new = self.A @ x_predict + self.B @ self.input
        next_state = self.state + (self.dt / 2.0) * (f_prev + f_new)
        self.state = next_state

        # 퍼블리시 명령
        pitch_cmd = float(self.input[0, 0]) * -0.005
        yaw_cmd = float(self.input[1, 0]) * 0.005
        roll_cmd = float(roll_command) * 0.005  # roll은 직접 퍼블리시

        self.pitch_pub.publish(Float64(data=pitch_cmd))
        self.yaw_pub.publish(Float64(data=yaw_cmd))
        self.roll_pub.publish(Float64(data=roll_cmd))

        self.get_logger().info(
            f"[State] θ={self.state[0,0]:.2f}, ψ={self.state[2,0]:.2f}, ϕ={self.state[4,0]:.2f} | "
            f"[Cmd] P={pitch_cmd:.2f}, Y={yaw_cmd:.2f}, R={roll_cmd:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ThreeDOFStateSpaceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
