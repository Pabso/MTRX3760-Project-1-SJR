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

# connecting to the turtlebot (for local network)
static ip (192.168.0.69) will have to be changed per network
ensure the tutlebot is set to wifi mode  
once the green ligh is flashing on the extension board  
$ ssh ubuntu@192.168.0.69



