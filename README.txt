Requirements:
==========
Use Robo Explorer with Wheels instead of Tracks:
Tracks are too inaccurate for Odometry.
I used the Setup from Mobile Robots 2.

Commands:
==========
-When running a component (eg. rviz) remote:
# export ROS_MASTER_URI=http://192.168.0.2:11311
# export ROS_IP=$OWN_IP
-Start Explorer:
# roslaunch explorer_configuration.launch
-Start Navigation Stack:
# roslaunch move_base.launch
-Start RViz:
# rosrun rviz rviz
-Manually setting of Translation/Rotation speed:
# rostopic pub -1 cmd_vel geometry_msgs/Twist '[0.0, 0, 0]' '[0, 0, 0.0]'

Notes:
==========
Laserscan is faked by the sonar sensor.
