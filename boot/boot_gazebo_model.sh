#!/bin/sh

roslaunch racecar_gazebo racecar_tunnel.launch &

sleep 3

roslaunch racecar_gazebo controller_manager.launch &

sleep 3

rosservice call /racecar/controller_manager/load_controller "name: 'left_rear_wheel_velocity_controller'"
rosservice call /racecar/controller_manager/load_controller "name: 'right_rear_wheel_velocity_controller'"
rosservice call /racecar/controller_manager/load_controller "name: 'left_front_wheel_velocity_controller'"
rosservice call /racecar/controller_manager/load_controller "name: 'right_front_wheel_velocity_controller'"
rosservice call /racecar/controller_manager/load_controller "name: 'left_steering_hinge_position_controller'"
rosservice call /racecar/controller_manager/load_controller "name: 'right_steering_hinge_position_controller'"


rosservice call /racecar/controller_manager/switch_controller "{start_controllers:\
['right_front_wheel_velocity_controller',\
'left_front_wheel_velocity_controller',\
'right_rear_wheel_velocity_controller',\
'right_rear_wheel_velocity_controller',\
'left_steering_hinge_position_controller',\
'right_steering_hinge_position_controller']}"
