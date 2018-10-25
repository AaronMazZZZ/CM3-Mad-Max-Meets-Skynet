#!/bin/sh

roslaunch map_server map_server_racecar.launch &

sleep 2

roslaunch amcl amcl.launch 




