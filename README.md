# MTRX3760-Project-1-SJR
Project 1: Hello, Turtlebots!, Autonomous Maze Solver

## Sequence
In every terminal!
source catkin/deval/setup.bash

# 1
export TURTLEBOT3_MODEL=burger  
roslaunch turtlebot3_gazebo turtlebot3_maze.launch  

# 2 
rosrun project1 wallFollower

# Helpful Commands
rosrun rqt_graph rqt_graph  
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

## connecting to the turtlebot (for local network)
follow steps outlined in turtlebots user guide.  
static ip (192.168.0.69) will have to be changed per network.   
ensure the tutlebot is set to wifi mode.  
once the green ligh is flashing on the extension board
you can connect to the turtlebot via:  
$ ssh ubuntu@192.168.0.69  
# Hotspot
PC static: 192.168.184.79  
Turtlebot Static: 192.168.184.69  


## Setting up node connections
A master roscore must be running on the host computer (laptop etc.)
ON the PC change hostname to ip4 adress found on ifconfig!! e.g   
export ROS_HOSTNAME=192.168.0.13
roscore will provide you with a ROS_MASTER_URI, this needs to be exported on the turtlebot:  
$ export ROS_MASTER_URI=http://192.168.0.13:11311/  
note that the url will be different for each instance of roscore.
Futhermore, the ROS_HOSTNAME must be set to the static ip adress given in the connecting to the turtlebot section e.g.  
export ROS_HOSTNAME=192.168.0.69  
run roswtf on the turtlebot to see any issues.  
See https://wiki.ros.org/ROS/NetworkSetup for more details  






