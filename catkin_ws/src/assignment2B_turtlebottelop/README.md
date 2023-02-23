# AuE8230Spring23_hw2b
This is the ROS project folder for the AuE8230 course hw2b. 

The video link is in the "videos" folder which is a public google drive folder you can check.
 
 
## Table of contents

- Teleop task
- Circle task
- Square task


## Teleop task

Open the terminal and run the following command

```Linux
 roscore
```

Then connect the turtlebot to the remote PC 

```Linux
 ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

Bring up basic packages to start turtlebot3 application

```Linux
 export TURTLEBOT3_MODEL=burger
 roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
Launch the turtlebot3_teleop_key node from your remote PC

```Linux
 export TURTLEBOT3_MODEL=burger
 roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Then your keyboard "WDSA" to control the turtlebot3.  


## Circle task

Execute the following commands

Open the terminal and run the following command

```Linux
 roscore
```

Then connect the turtlebot to the remote PC 

```Linux
 ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

Bring up basic packages to start turtlebot3 application

```Linux
 export TURTLEBOT3_MODEL=burger
 roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
Launch the turtlebot3_teleop_key node from your remote PC

```Linux
 export TURTLEBOT3_MODEL=burger
```
To run the package and start the node, we use a different launch file to start the node using the following commands based on our requirement.
Choose launch file based on what maneouver is expected.

 This code is used to run the bot with the code circle.py based on the launch code specified
 
```
 roslaunch assignment2B_turtlebottelop circle_fast.launch
```
 This runs the bot in a circle with a linear velocity of 0.4 and angular velocity of 1.2
 
```
 roslaunch assignment2B_turtlebottelop circle_medium.launch
```
 This runs the bot in a circle with a linear velocity of 0.2 and angular velocity of 0.6
 
```
 roslaunch assignment2B_turtlebottelop circle_slow.launch
```
This runs the bot in a circle with a linear velocity of 0.1 and angular velocity of 0.3

Analysis

The bot completes the circle as expected from the command.
In case of higher speeds, the radius is smaller even though the ratio of Linear to Angular Velocity is the same.
The higher speeds made the turtle cover more distance for the same program execution time.
  
## Square task

Execute the following commands

Open the terminal and run the following command

```Linux
 roscore
```

Then connect the turtlebot to the remote PC 

```Linux
 ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

Bring up basic packages to start turtlebot3 application

```Linux
 export TURTLEBOT3_MODEL=burger
 roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
Launch the turtlebot3_teleop_key node from your remote PC

```Linux
 export TURTLEBOT3_MODEL=burger
```
To run the package and start the node, we use a different launch file to start the node using the following commands based on our requirement.
Choose launch file based on what maneouver is expected.
 
```Linux 
 roslaunch assignment2B_turtlebottelop move.launch code:=square
```
 This code is used to run the bot with the code square.py based on the launch code specified

``` 
 roslaunch assignment2B_turtlebottelop square_slow.launch
```
This runs the bot in a square linear velocity of 0.4 and angular velocity of 0.4

```
 roslaunch assignment2B_turtlebottelop square_medium.launch
```
This runs the bot in a square linear velocity of 0.6 and angular velocity of 0.4

```  
 roslaunch assignment2B_turtlebottelop square_fast.launch
```
This runs the bot in a square linear velocity of 1.1 and angular velocity of 0.4

Analysis

The bot completes the square as expected from the command.
We had corrected for the angular displacement (0.05 radian) based on error noticed to complete the 90 degree turn.
In case of higher speeds, the bot takes time to accelerate to the desired speed and hence has shorter distance travelled in time-based open loop control.
