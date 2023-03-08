# AuE8230Spring23_hw3A
This is the ROS project folder for the AuE8230 course hw3A. 

The video link is in the "videos" folder which is a public google drive folder you can check.
 
 
## Table of contents

- Wall following
- Obstacle avoidance
- Obstacle avoidance in real world
- Emergency braking


## Wall following

Execute the following commands
```bash
 roslaunch assignment3a_team4 wall_following.launch
```
The simulation will start to run.

## Obstacle avoidance

Execute the following commands

```bash
 roslaunch assignment3a_team4 wander.launch
```
The simulation will start to run.

  
## Obstacle avoidance in real world
Open the remote PC terminal and run the following command

```bash
 roscore
```

Then connect the turtlebot to the remote PC 

```bash
 ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

Bring up basic packages to start turtlebot3 application in the turtlebot terminal

```bash
 export TURTLEBOT3_MODEL=burger
 roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
When the turtlebot and remote PC has been successfully connected, put the turtlebot in a suitable environment and execute the following commands

```bash
 export TURTLEBOT3_MODEL=burger
 roslaunch assignment3a_team4 real_wander.launch
```
The turtlebot will wander in the real world.

## Emergency braking

Execute the following commands

```bash
 roslaunch assignment3a_team4 emergency_braking.launch
```
The simulation will start to run.






