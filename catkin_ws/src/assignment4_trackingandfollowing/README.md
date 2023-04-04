# AuE8230Spring23_hw4
This is the ROS project folder for the AuE8230 course hw4. 

The video link is in the "videos" folder which is a public google drive folder you can check.

## Table of contents
The project contains majorly 2 tasks, 
- Line following using Blob tracking or Hough transforms
- April tag tracking

It was expected to have both the tasks implemented on real Turtlebot and only simulation for task 1 on Gazebo simulator.

## Following are the terminal commands
For both the tasks, follow below terminal commands. Open the terminal and run the following command.
```Linux
 	roscore
```
Then connect the turtlebot to the remote PC. 
```Linux
 ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

Bring up basic packages to start turtlebot3 application.
```Linux
 	roslaunch turtlebot3_bringup turtlebot3_robot. launch
```

Launch the launch files from your remote PC.
```Linux
 	export TURTLEBOT3_MODEL=burger
```

##publish the image detection and April tag detection from the image to turtlebot
##only for April tag detection task (library download for tag pose detection)
```Linux
roslaunch apriltag_ros continuous_detection.launch
```

#For line following
```Linux
roslaunch assignment4_trackingandfollowing turtlebot3_follow_line.launch
```

#For April Tag
```Linux
roslaunch assignment4_trackingandfollowing turtlebot3_follow_apriltag.launch
```


### Task 1: Line following using Blob tracking
Blob tracking was used to detect the lines. The turtlebot will wander randomly till it finds the line and then follow that line.

The code, follow_line_step_hsv.py was used for this task. This Python script will control a robot so that it follows a yellow line detected by a camera. The script uses a PID controller to adjust the robot's movement based on the current position of the line.

### Task 2: April Tag detection
The python file follow_april_tag.py was used for this task. A class called "tagFollow" is defined that contains the following methods:

•	The constructor "init()" initializes the ROS node, creates a publisher to publish velocity commands, creates a subscriber to receive AprilTag detection information, and initializes the x and z variables to zero.
•	The "tag_update()" method is the callback function that updates the x and z variables with the position of the detected AprilTag.
•	The "follow()" method is used to set the linear and angular velocities of the robot based on the position of the detected AprilTag. It uses proportional control to set the velocities.

Challenges faced:

•	During April Tag detection, the data transfer rate from turtlebot3 to remote PC was very slow and therefore there was a difficulty for the detection of tag pose. This was solved by using “/image_mono/compressed" instead of “/camera/image_color”.

It also got solved by connecting to local WIFI router connection.

•	For the line following task on real turtlebot, initially the turtlebot could not follow the line. Possible reasons we could figure out where the camera not able to locate the centroid appropriately and PID controller not working appropriately. Both the issues were addressed by tuning the PID controller and using proper light to focus on the white line.
