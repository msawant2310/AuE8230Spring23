run step by step
PC:
roscore

PBC:
roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch

PBC:
roslaunch turtlebot3_bringup turtlebot3_robot.launch

PC:
roslaunch apriltag_ros continuous_detection.launch 

roslaunch assignment4_trackingandfollowing turtlebot3_follow_apriltag.launch