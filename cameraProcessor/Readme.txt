##Camera NODE##

Detects if the majority color on the screen is 70% of 
the screen based on a threshold of 70% red on each pixel.

These values can be changed in camera.h, simialrly the 
subscribe path can be changed in main.cpp

TESTING COMMANDS:

/OPEN GAZEBO FOR SIMULATION
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 

/RUN NODE
rosrun cameraTester cameraTester_node


*****
AUTHOR: STEPHEN CAPAR
*****