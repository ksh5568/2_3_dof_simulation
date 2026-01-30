#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class BladeCommandPublisher(Node):
    def __init__(self):
        super().__init__('blade_start_node')

        # 퍼블리셔 설정
        self.pub1 = self.create_publisher(Float64, '/blade_1_cmd/command', 10)
        self.pub2 = self.create_publisher(Float64, '/blade_2_cmd/command', 10)
        self.pub3 = self.create_publisher(Float64, '/blade_3_cmd/command', 10)
        self.pub4 = self.create_publisher(Float64, '/blade_4_cmd/command', 10)

        # 0.1초마다 퍼블리시 (10Hz)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 블레이드 속도 설정 (예: 1.0 rad/s)
        self.speed1 = -50.0
        self.speed2 = -50.0 #cw
        self.speed3 = 50.0
        self.speed4 = 50.0

    def timer_callback(self):
        msg1 = Float64()
        msg1.data = self.speed1
        self.pub1.publish(msg1)

        msg2 = Float64()
        msg2.data = self.speed2
        self.pub2.publish(msg2)

        msg3 = Float64()
        msg3.data = self.speed3
        self.pub3.publish(msg3)

        msg4 = Float64()
        msg4.data = self.speed4
        self.pub4.publish(msg4)

        # self.get_logger().info('Publishing blade speeds: %.2f, %.2f, %.2f, %.2f' %
        #                        (self.speed1, self.speed2, self.speed3, self.speed4))


def main(args=None):
    rclpy.init(args=args)
    node = BladeCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

