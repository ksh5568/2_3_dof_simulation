import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import math
import os
import csv
from datetime import datetime
from scipy.spatial.transform import Rotation as R

class RealtimeRPYPlotter(Node):
    def __init__(self):
        super().__init__('realtime_rpy_plotter')

        # 데이터 버퍼 (최근 200개)
        self.time_data = deque(maxlen=200)
        self.roll_data = deque(maxlen=200)
        self.pitch_data = deque(maxlen=200)
        self.yaw_data = deque(maxlen=200)
        self.counter = 0

        # 초기값
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # 로그 저장 준비
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = os.path.expanduser("/home/kimsh/ros2_ws/src/my_robot/my_robot/src")
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_path = os.path.join(self.log_dir, f"3dof_rpy_log_{timestamp}.csv")

        with open(self.log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_step', 'roll_deg', 'pitch_deg', 'yaw_deg'])

        # IMU 구독
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

    def imu_callback(self, msg):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        rpy = R.from_quat(quat).as_euler('xyz', degrees=True)
        self.roll = rpy[0]
        self.pitch = rpy[2]
        self.yaw = rpy[1]

    def update_plot(self, frame):
        self.counter += 1
        self.time_data.append(self.counter)
        self.roll_data.append(self.roll)
        self.pitch_data.append(self.pitch)
        self.yaw_data.append(self.yaw)

        # 데이터 저장
        with open(self.log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.counter, self.roll, self.pitch, self.yaw])

        # 그래프 초기화
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()

        # Roll plot
        self.ax1.plot(self.time_data, self.roll_data, color='red')
        self.ax1.axhline(0, color='gray', linestyle='--')
        self.ax1.set_ylabel('Roll (°)')
        self.ax1.set_title('Roll')
        self.ax1.grid(True)

        # Pitch plot
        self.ax2.plot(self.time_data, self.pitch_data, color='blue')
        self.ax2.axhline(0, color='gray', linestyle='--')
        self.ax2.set_ylabel('Pitch (°)')
        self.ax2.set_title('Pitch')
        self.ax2.grid(True)

        # Yaw plot
        self.ax3.plot(self.time_data, self.yaw_data, color='green')
        self.ax3.axhline(0, color='gray', linestyle='--')
        self.ax3.set_xlabel('Time Step (s)')
        self.ax3.set_ylabel('Yaw (°)')
        self.ax3.set_title('Yaw')
        self.ax3.grid(True)

    def start_plotting(self):
        fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(12, 10))
        ani = animation.FuncAnimation(fig, self.update_plot, interval=200)
        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = RealtimeRPYPlotter()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    node.start_plotting()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
