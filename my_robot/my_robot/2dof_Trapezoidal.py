import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class StateSpaceTrapezoidalController(Node):
    def __init__(self):
        super().__init__('state_space_trapezoidal_controller')

        self.pitch_pub = self.create_publisher(Float64, '/pitch_wheel/command', 10)
        self.yaw_pub = self.create_publisher(Float64, '/yaw_wheel/command', 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_sensor_data)

        # 시스템 파라미터 (예시값, 필요시 수정)
        self.dt = 0.05  # 20 Hz
        self.state = np.zeros((4, 1))  # [theta, theta_dot, psi, psi_dot]
        self.prev_state = np.zeros((4, 1))
        self.input = np.zeros((2, 1))  # [Vp, Vy]

        # 시스템 행렬 (실측 또는 문헌 기반 값 필요)
        self.A = np.array([
            [0, 1, 0, 0],
            [-2.7452, -0.2830, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 0, -0.2701]
        ])
        self.B = np.array([
            [0, 0],
            [37.2021, 0],
            [0, 0],
            [0, 7.461]
        ])
        self.C = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0]
        ])
        self.D = np.array([
            [0, 0],
            [0, 0]
        ])
        self.pitch_offset = None
        self.yaw_offset = None

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('🚁 Trapezoidal State-Space Controller Initialized')

    def imu_callback(self, msg):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        rpy = R.from_quat(quat).as_euler('xyz', degrees=True)

        pitch = -rpy[2]
        yaw = rpy[1]

        if self.pitch_offset is None:
            self.pitch_offset = pitch
            self.yaw_offset = yaw
            self.get_logger().info(f'IMU offset set: pitch={pitch:.2f}, yaw={yaw:.2f}')
            return

        self.measured_output = np.array([
            [pitch - self.pitch_offset],
            [yaw - self.yaw_offset]
        ])

    def control_loop(self):
        if not hasattr(self, 'measured_output'):
            self.get_logger().warn('Waiting for IMU...')
            return

        # 단순 목표값
        y_target = np.array([[0.0], [0.0]])

        # 추정 출력 오차
        y_error = y_target - self.measured_output

        # 출력 오차를 단순히 각속도 목표로 변환 (비례 제어)
        self.input[0, 0] = 2.0 * y_error[0, 0]
        self.input[1, 0] = 2.0 * y_error[1, 0]

        # Trapezoidal 상태 예측
        f_prev = self.A @ self.state + self.B @ self.input
        x_predict = self.state + self.dt * f_prev  # 예측값으로 f_new 근사
        f_new = self.A @ x_predict + self.B @ self.input
        next_state = self.state + (self.dt / 2.0) * (f_prev + f_new)

        self.prev_state = self.state.copy()
        self.state = next_state

        # 제어 입력 계산 및 퍼블리시
        pitch_cmd = float(self.input[0, 0]) * -0.005
        yaw_cmd = float(self.input[1, 0]) * 0.005
        self.pitch_pub.publish(Float64(data=pitch_cmd))
        self.yaw_pub.publish(Float64(data=yaw_cmd))

        self.get_logger().info(
            f"State: pitch={self.state[0,0]:.2f}, yaw={self.state[2,0]:.2f} | "
            f"Output Cmd: P={pitch_cmd:.2f}, Y={yaw_cmd:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = StateSpaceTrapezoidalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
