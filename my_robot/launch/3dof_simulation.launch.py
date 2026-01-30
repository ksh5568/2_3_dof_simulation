import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():
    sdf_model_path = "/home/kimsh/ros2_ws/src/my_robot/sdf/DOF_Model/3dof_simulation_model.sdf"

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
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
            ],
            output='screen'
        ),
    ])

