gz sim /home/kimsh/ros2_ws/src/my_robot/sdf/dof_simulation.sdf

gz topic -e -t /imu

gz topic -l

gz topic -t "/pitch_wheel/command" -m gz.msgs.Double -p "data: -0.1"



gz sim /home/kimsh/ros2_ws/src/pan_tilt/sdf pan_tilt_system.sdf

gz topic -t "/pan/command" -m gz.msgs.Double -p "data: -0.3"
gz topic -t "/tilt/command" -m gz.msgs.Double -p "data: -0.3"

gz topic -t "/pan/command" -m gz.msgs.Double -p "data: 0.3"
gz topic -t "/tilt/command" -m gz.msgs.Double -p "data: 0.3"
