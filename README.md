# MTRX3760-Project-1-SJR
Project 1: Hello, Turtlebots!, Autonomous Maze Solver

# Sequence
source catkin/deval/setup.bash

export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

rosrun pr2_controller_manager pr2_controller_manager list

rosrun project1 wallFollower
