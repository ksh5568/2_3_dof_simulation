🚀 My Robot Simulation Package

ROS2 기반 다자유도(DOF) 시뮬레이션 및 Pan-Tilt 시스템 통합 패키지입니다.
Gazebo Sim(GZ Sim)과 ros_gz_bridge를 활용해 로봇 제어 및 센서 데이터를 ROS2 환경에서 사용 가능합니다.

📂 패키지 구조
my_robot/
├── setup.py                     # 패키지 설정:contentReference[oaicite:0]{index=0}
├── setup.cfg
├── package.xml
├── launch/
│   ├── 2dof_simulation.launch.py    # 2DOF 모델 실행:contentReference[oaicite:1]{index=1}
│   ├── 3dof_simulation.launch.py    # 3DOF 모델 실행:contentReference[oaicite:2]{index=2}
│   ├── ndof_simulation.launch.py    # nDOF 모델 실행:contentReference[oaicite:3]{index=3}
│   ├── pan_tilt_system.launch.py    # Pan/Tilt 시스템 실행:contentReference[oaicite:4]{index=4}
│   ├── pan_tilt_system_bridge.launch.py # Pan/Tilt ROS-GZ 브리지:contentReference[oaicite:5]{index=5}
└── sdf/                          # 시뮬레이션용 SDF 모델들

⚙️ 주요 기능

다자유도 시뮬레이션

2DOF, 3DOF, nDOF 시뮬레이션 모델 실행 가능

gz sim을 통해 물리 시뮬레이션 수행

ROS2 노드와 GZ Sim 토픽을 ros_gz_bridge로 연결

Pan/Tilt 시스템

카메라, 라이다, IMU 센서 연동

/pan/command, /tilt/command 토픽으로 제어 가능

pan_tilt_system_bridge.launch

Gazebo 센서 데이터 → ROS2 메시지 변환 (LaserScan, PointCloud2, Image, Imu 등)

블레이드 제어

nDOF 모델에서 추가적으로 블레이드 제어 가능 (/blade_1_cmd, /blade_2_cmd 등)

ndof_simulation.launch

blade_start 노드 실행 포함

▶ 실행 방법
1. 2DOF 시뮬레이션
ros2 launch my_robot 2dof_simulation.launch.py

2. 3DOF 시뮬레이션
ros2 launch my_robot 3dof_simulation.launch.py

3. nDOF 시뮬레이션
ros2 launch my_robot ndof_simulation.launch.py

4. Pan/Tilt 시스템
ros2 launch my_robot pan_tilt_system.launch.py

5. Pan/Tilt 시스템 + ROS-GZ 브리지
ros2 launch my_robot pan_tilt_system_bridge.launch.py

🖼️ 토픽 브리지 예시

/keyboard/keypress → std_msgs/msg/Int32

/pan/command, /tilt/command → std_msgs/msg/Float64

/scan, /scan/points → LaserScan, PointCloud2

/image_raw → Image, CameraInfo

/imu, /navsat → Imu, NavSatFix

🛠️ 개발 환경

OS: Ubuntu 22.04

ROS2: Humble

Simulator: Gazebo Sim (gz sim)

Bridge: ros_gz_bridge

📧 Maintainer

Author: 김성현

Email: kimsh315331@gmail.com

🔹 DOF 시뮬레이션 실행 및 제어
# DOF 시뮬레이션 실행 (기본 모델 실행)
gz sim /home/kimsh/ros2_ws/src/my_robot/sdf/dof_simulation.sdf


➡️ dof_simulation.sdf 모델을 Gazebo Sim에서 실행.

# 3DOF 시뮬레이션 실행
gz sim /home/kimsh/ros2_ws/src/my_robot/sdf/DOF_Model/3dof_simulation_model.sdf


➡️ 3자유도(3DOF) 모델을 로딩하여 시뮬레이션 시작.

# IMU 토픽의 메시지를 실시간 확인
gz topic -e -t /imu


➡️ /imu 토픽 데이터를 Echo(출력). IMU 센서에서 오는 관성 데이터 확인 가능.

# 현재 Gazebo에서 발행 중인 모든 토픽 리스트 확인
gz topic -l


➡️ Gazebo에서 사용 가능한 토픽 목록 출력.

⚙️ DOF 제어 명령
# Pitch 휠 제어 (음수 → 반대 방향 회전)
gz topic -t "/pitch_wheel/command" -m gz.msgs.Double -p "data: -0.1"


➡️ /pitch_wheel/command 토픽에 -0.1 값을 발행 → pitch 휠을 음의 방향으로 제어.

# Roll 휠 제어 (음수 → 반대 방향 회전)
gz topic -t "/roll_wheel/command" -m gz.msgs.Double -p "data: -0.2"


➡️ /roll_wheel/command 토픽에 -0.2 값을 발행 → roll 휠을 음의 방향으로 제어.

# 블레이드(예: 프로펠러) 제어
gz topic -t "/blade_1_cmd/command" -m gz.msgs.Double -p "data: 0.1"


➡️ /blade_1_cmd/command 토픽에 0.1 값을 발행 → 블레이드 1을 양의 속도로 회전.

🔹 Pan-Tilt 시스템 실행 및 제어
# Pan-Tilt 시스템 시뮬레이션 실행
gz sim /home/kimsh/ros2_ws/src/pan_tilt/sdf pan_tilt_system.sdf


➡️ Pan-Tilt 시스템 모델을 Gazebo Sim에서 실행.

⚙️ Pan-Tilt 제어 (Gazebo 명령)
# Pan, Tilt를 음수 값으로 제어 (왼쪽/아래 방향)
gz topic -t "/pan/command" -m gz.msgs.Double -p "data: -0.3"
gz topic -t "/tilt/command" -m gz.msgs.Double -p "data: -0.3"


➡️ Pan과 Tilt를 동시에 음의 값으로 움직여 카메라 방향을 좌하향으로 조정.

# Pan, Tilt를 양수 값으로 제어 (오른쪽/위 방향)
gz topic -t "/pan/command" -m gz.msgs.Double -p "data: 0.3"
gz topic -t "/tilt/command" -m gz.msgs.Double -p "data: 0.3"


➡️ Pan과 Tilt를 동시에 양의 값으로 움직여 카메라 방향을 우상향으로 조정.

🔹 ROS2 ↔ Gazebo 브리지 실행
# 카메라 이미지 & CameraInfo 토픽 브리지
ros2 run ros_gz_bridge parameter_bridge \
  /world/sensor_world/model/pan_tilt_system/link/camera_link/sensor/camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/sensor_world/model/pan_tilt_system/link/camera_link/sensor/camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo


➡️ Gazebo에서 발생하는 카메라 이미지/CameraInfo 토픽을 ROS2 메시지(Image, CameraInfo)로 변환.
RViz나 ROS2 노드에서 직접 카메라 데이터를 활용 가능.

# 라이다 센서 브리지
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan


➡️ Gazebo의 /lidar 토픽을 ROS2의 LaserScan 메시지로 변환.
ROS2 기반 SLAM, Obstacle Detection 등에서 사용 가능.

🔹 ROS2 명령어 기반 제어
# Tilt를 ROS2에서 직접 제어
ros2 topic pub /tilt/command std_msgs/msg/Float64 "{data: -0.1}"


➡️ ROS2에서 /tilt/command 토픽으로 -0.1 값 발행 → 카메라 Tilt 축을 음의 방향으로 제어.