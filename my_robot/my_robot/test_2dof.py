import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from scipy.spatial.transform import Rotation as R

class ImuRPYListener(Node):
    def __init__(self):
        super().__init__('imu_rpy_listener')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        # --- 쿼터니언 ---
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        quat = [x, y, z, w]

        # --- Roll & Pitch 계산 (오일러 변환) ---
        rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
        roll_deg = rpy[0]
        pitch_deg = rpy[2]
        yaw_deg = rpy[1]

        # --- 출력 ---
        self.get_logger().info(
            f"Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = ImuRPYListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

