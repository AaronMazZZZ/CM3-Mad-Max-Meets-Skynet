#!/bin/sh


#pid linear
#rosrun trajectory_tracking control.py pid linear 12

#racecar pid follow global path 
rosrun trajectory_tracking control.py racecar_controller racecar 15
