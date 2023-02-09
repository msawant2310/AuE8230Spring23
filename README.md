# AuE8230Spring23
This is the ROS project folder for the AuE8230 course. 

All the videos are in the corresponding package folder.
 
 
## Table of contents

- Prerequisite
- Assignment1b
- Assignment1c


## Prerequisite

You need to change the right IP address in the ".bashrc" file and run the 'catkin_make' and 'source devel/setup.bash'

## Assignment1b

Assignment1b is created by Siqi Zheng.
For the Assignment1b, it is in the package assignment1b folder under the directory: '~/catkin_ws/src/assignment1b'


To run the package and start the node, using the following three commands for the three tasks in the "catkin_ws" path:

'''Python
 roslaunch assignment1b circle.launch
 roslaunch assignment1b square_openloop.launch
 roslaunch assignment1b square_closedloop.launch
'''


## Assignment1c

Assignment1c is created by the group4.
For the Assignment1c, it is in the package assignment1c folder under the directory '~/catkin_ws/src/assignment1c'


To run the package and start the node, using the following three commands for the three tasks in the "catkin_ws" path:

'''Linux
 roslaunch assignment1c circle_fast.launch
 roslaunch assignment1c circle_medium.launch
 roslaunch assignment1c circle_slow.launch
 roslaunch assignment1c square_fast.launch
 roslaunch assignment1c square_medium.launch
 roslaunch assignment1c square_slow.launch
 roslaunch assignment1c move.launch code:=square
 roslaunch assignment1c move.launch code:=circle
'''



