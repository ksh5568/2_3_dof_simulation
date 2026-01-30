import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import math
from rclpy.qos import qos_profile_sensor_data

class DOFHelicopterController(Node):
    def __init__(self):
        super().__init__('dof_helicopter_controller')

        self.pitch_pub = self.create_publisher(Float64, '/pitch_wheel/command', 10)
        self.yaw_pub = self.create_publisher(Float64, '/yaw_wheel/command', 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_sensor_data)

        # 제어 주기
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

        # 초기 상태 및 오프셋
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.pitch_offset = None
        self.yaw_offset = None

        # PID 제어기 설정 (조정됨)
        self.pitch_pid = {'Kp': 3.0, 'Ki': 0.1, 'Kd': 1.0, 'integral': 0.0, 'prev_error': 0.0}
        self.yaw_pid = {'Kp': 3.0, 'Ki': 0.1, 'Kd': 1.0, 'integral': 0.0, 'prev_error': 0.0}

        self.get_logger().info('🚁 DOF Helicopter Controller Initialized (with scipy RPY parsing)')

    def imu_callback(self, msg):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]

        # 쿼터니언 → RPY (XYZ 기준)
        rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
        roll_deg = rpy[0]
        pitch_deg = rpy[2]
        yaw_deg = rpy[1]

        # 초기 offset 설정
        if self.pitch_offset is None:
            self.pitch_offset = -pitch_deg
            self.yaw_offset = yaw_deg
            self.get_logger().info(f'🔧 IMU offset calibrated: pitch={self.pitch_offset:.2f}, yaw={self.yaw_offset:.2f}')

        self.roll = roll_deg  # 사용 안해도 저장해 둠
        self.pitch = -pitch_deg - self.pitch_offset
        self.yaw = yaw_deg - self.yaw_offset

    def compute_pid(self, pid, error):
        # Deadband 적용
        if error is None or math.isnan(error) or abs(error) < 1.0:
            return 0.0

        pid['integral'] += error * self.dt
        pid['integral'] = max(-50, min(50, pid['integral']))  # anti-windup

        derivative = (error - pid['prev_error']) / self.dt
        pid['prev_error'] = error

        output = (
            pid['Kp'] * error +
            pid['Ki'] * pid['integral'] +
            pid['Kd'] * derivative
        )

        output = max(-8.0, min(8.0, output))  # 제어 출력 제한
        return output

    def control_loop(self):
        if self.pitch is None or self.yaw is None:
            self.get_logger().warn('Waiting for IMU orientation...')
            return

        pitch_target = 0.0
        yaw_target = 0.0

        pitch_error = pitch_target - self.pitch
        yaw_error = yaw_target - self.yaw

        pitch_output = self.compute_pid(self.pitch_pid, pitch_error) * -0.08
        yaw_output = self.compute_pid(self.yaw_pid, yaw_error) * 0.08

        # 제어 명령 발행
        self.pitch_pub.publish(Float64(data=pitch_output))
        self.yaw_pub.publish(Float64(data=yaw_output))

        # 상태 출력
        self.get_logger().info(
            f'🎯 Target=0°, Pitch={self.pitch:.2f}°, Yaw={self.yaw:.2f}°, '
            f'Error → P={pitch_error:.2f}, Y={yaw_error:.2f}, '
            f'Output → P={pitch_output:.3f}, Y={yaw_output:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    controller = DOFHelicopterController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
