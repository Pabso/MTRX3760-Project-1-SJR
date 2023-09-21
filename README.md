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




