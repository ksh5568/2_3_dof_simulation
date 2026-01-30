import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():
    sdf_model_path = "/home/kimsh/ros2_ws/src/my_robot/sdf/DOF_Model/ndof_simulation_model.sdf"

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['gz', 'sim', sdf_model_path],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/keyboard/keypress@std_msgs/msg/Int32@gz.msgs.Int32',
                '/pitch_wheel/command@std_msgs/msg/Float64@gz.msgs.Double',
                '/yaw_wheel/command@std_msgs/msg/Float64@gz.msgs.Double',
                '/roll_wheel/command@std_msgs/msg/Float64@gz.msgs.Double',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/blade_1_cmd/command@std_msgs/msg/Float64@gz.msgs.Double',
                '/blade_2_cmd/command@std_msgs/msg/Float64@gz.msgs.Double',
                '/blade_3_cmd/command@std_msgs/msg/Float64@gz.msgs.Double',
                '/blade_4_cmd/command@std_msgs/msg/Float64@gz.msgs.Double'                
            ],
            output='screen'
        ),

        # blade_start 노드 실행
        launch_ros.actions.Node(
            package='my_robot',                # ← blade_start.py가 속한 패키지 이름
            executable='blade_start',         # ← setup.py entry_point에서 정의된 이름
            name='blade_start_node',          # ← 노드 이름 (선택사항)
            output='screen'
        )
    ])

