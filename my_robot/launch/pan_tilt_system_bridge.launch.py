import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/keyboard/keypress@std_msgs/msg/Int32@gz.msgs.Int32',
                '/pan/command@std_msgs/msg/Float64@gz.msgs.Double',
                '/tilt/command@std_msgs/msg/Float64@gz.msgs.Double',
                "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
                '/image_raw@sensor_msgs/msg/Image@gz.msgs.Image', 
                '/image_raw@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/lidar_imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/camera\_imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",

            ],
            output='screen'
        ),
    ])

