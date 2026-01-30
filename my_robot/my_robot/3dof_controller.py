import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import math

def euler_from_quaternion(quat):
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, +1.0), -1.0)
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

class DOFHelicopterController(Node):
    def __init__(self):
        super().__init__('dof_helicopter_controller')

        # 퍼블리셔 생성
        self.roll_pub = self.create_publisher(Float64, '/roll_wheel/command', 10)
        self.pitch_pub = self.create_publisher(Float64, '/pitch_wheel/command', 10)
        self.yaw_pub = self.create_publisher(Float64, '/yaw_wheel/command', 10)

        # 초기값 발행
        init_msg = Float64()
        init_msg.data = 0.1
        self.roll_pub.publish(init_msg)
        self.pitch_pub.publish(init_msg)
        self.yaw_pub.publish(init_msg)

        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.timer = self.create_timer(1.0, self.control_loop)

        # 상태 변수
        self.roll = None
        self.pitch = None
        self.yaw = None

        # PID 설정
        self.roll_pid = {'Kp': 3.0, 'Ki': 0.0, 'Kd': 0.5, 'integral': 0.0, 'prev_error': 0.0}
        self.pitch_pid = {'Kp': 3.0, 'Ki': 0.0, 'Kd': 0.5, 'integral': 0.0, 'prev_error': 0.0}
        self.yaw_pid = {'Kp': 3.0, 'Ki': 0.0, 'Kd': 0.5, 'integral': 0.0, 'prev_error': 0.0}

        self.get_logger().info('🚁 DOF Helicopter Controller with Roll-Pitch-Yaw Initialized!')

    def imu_callback(self, msg):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        self.roll = math.degrees(roll)
        self.pitch = -math.degrees(pitch)
        self.yaw = math.degrees(yaw)

        self.get_logger().info(f'IMU - Roll: {self.roll:.2f}°, Pitch: {self.pitch:.2f}°, Yaw: {self.yaw:.2f}°')

    def compute_pid(self, pid, error):
        if error is None or math.isnan(error):
            self.get_logger().warn('Invalid PID error detected, resetting to 0.0')
            error = 0.0

        pid['integral'] += error * 0.1
        pid['integral'] = max(-30, min(30, pid['integral']))
        derivative = (error - pid['prev_error']) / 0.1
        pid['prev_error'] = error
        output = (pid['Kp'] * error) + (pid['Ki'] * pid['integral']) + (pid['Kd'] * derivative)

        return max(-3, min(3, output))

    def control_loop(self):
        if self.roll is None or self.pitch is None or self.yaw is None:
            self.get_logger().warn('Waiting for orientation data...')
            return

        # 목표값
        roll_target = 0.0
        pitch_target = 0.0
        yaw_target = 0.0

        # 오차 계산
        roll_error = roll_target - self.roll
        pitch_error = pitch_target - self.pitch
        yaw_error = yaw_target - self.yaw

        # PID 계산
        roll_output = self.compute_pid(self.roll_pid, roll_error)
        pitch_output = self.compute_pid(self.pitch_pid, pitch_error)
        yaw_output = self.compute_pid(self.yaw_pid, yaw_error)

        # 메시지 생성
        roll_msg = Float64()
        pitch_msg = Float64()
        yaw_msg = Float64()

        roll_msg.data = float(roll_output * 0.06)
        pitch_msg.data = float(pitch_output * 0.06)
        yaw_msg.data = float(yaw_output * 0.06)

        # 퍼블리시
        self.roll_pub.publish(roll_msg)
        self.pitch_pub.publish(pitch_msg)
        self.yaw_pub.publish(yaw_msg)

        self.get_logger().info(f'Control Output - Roll: {roll_output:.2f}, Pitch: {pitch_output:.2f}, Yaw: {yaw_output:.2f}')

def main(args=None):
    rclpy.init(args=args)
    controller = DOFHelicopterController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
