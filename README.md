# MTRX3760-Project-1-SJR
Project 1: Hello, Turtlebots!, Autonomous Maze Solver

# Sequence
source catkin/deval/setup.bash

export TURTLEBOT3_MODEL=burger
// roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_gazebo turtlebot3_maze.launch

rosrun project1 wallFollower


