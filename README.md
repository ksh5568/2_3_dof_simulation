# 🚀 My Robot Simulation Package

ROS 2 기반 다자유도(DOF) 로봇 시뮬레이션 및 Pan-Tilt 시스템 통합 패키지입니다.
Gazebo Sim(GZ Sim)과 `ros_gz_bridge`를 활용하여 **로봇 제어**를 ROS 2 환경에서 사용할 수 있습니다.

---

## ✅ Features

### 1) 다자유도(DOF) 시뮬레이션

* 2DOF / 3DOF / nDOF 모델 실행 지원
* `gz sim` 기반 물리 시뮬레이션
* `ros_gz_bridge`로 Gazebo ↔ ROS 2 토픽 연동

### 2) 블레이드 제어 (nDOF)

* nDOF 모델에서 블레이드 제어 지원
  (예: `/blade_1_cmd/command`, `/blade_2_cmd/command` 등)
* `ndof_simulation.launch.py`에서 `blade_start` 노드 실행 포함

---

## 📂 Package Structure

```bash
my_robot/
├── setup.py
├── setup.cfg
├── package.xml
├── launch/
│   ├── 2dof_simulation.launch.py
│   ├── 3dof_simulation.launch.py
│   ├── ndof_simulation.launch.py
│   ├── pan_tilt_system.launch.py
│   └── pan_tilt_system_bridge.launch.py
└── sdf/
    ├── dof_simulation.sdf
    └── DOF_Model/
        └── 2dof_simulation_model.sdf
        └── 3dof_simulation_model.sdf
        └── ndof_simulation_model.sdf
```

---

## 🧩 Requirements

* OS: Ubuntu 22.04
* ROS 2: Humble
* Simulator: Gazebo Sim (`harmonic`)
* Bridge: `ros_gz_bridge`

---

## 🛠 Build

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot
source install/setup.bash
```

---

## ▶️ Quick Start (Launch)

### 1) 2DOF 시뮬레이션

```bash
ros2 launch my_robot 2dof_simulation.launch.py
```

### 2) 3DOF 시뮬레이션

```bash
ros2 launch my_robot 3dof_simulation.launch.py
```

### 3) nDOF 시뮬레이션

```bash
ros2 launch my_robot ndof_simulation.launch.py
```

## 🔹 DOF Simulation (Gazebo Only)

> 워크스페이스 경로는 사용 환경에 맞게 수정하세요.

### 기본 DOF 모델 실행

```bash
gz sim ~/ros2_ws/src/my_robot/sdf/DOF_Model/2dof_simulation_model.sdf
```

### 3DOF 모델 실행

```bash
gz sim ~/ros2_ws/src/my_robot/sdf/DOF_Model/3dof_simulation_model.sdf
```

### Gazebo 토픽 확인

```bash
# 발행 중인 토픽 리스트
gz topic -l

# IMU 토픽 메시지 확인 (예: /imu)
gz topic -e -t /imu
```

---

## ⚙️ DOF Control Commands (Gazebo Topic Pub)

### Pitch Wheel 제어 (음수 → 반대 방향 회전)

```bash
gz topic -t "/pitch_wheel/command" -m gz.msgs.Double -p "data: -0.1"
```

### Roll Wheel 제어 (음수 → 반대 방향 회전)

```bash
gz topic -t "/roll_wheel/command" -m gz.msgs.Double -p "data: -0.2"
```

### Blade 제어 (예: 블레이드 1)

```bash
gz topic -t "/blade_1_cmd/command" -m gz.msgs.Double -p "data: 0.1"
```

## 👤 Maintainer

* Author: 김성현
* Email: [kimsh315331@gmail.com](mailto:kimsh315331@gmail.com)
