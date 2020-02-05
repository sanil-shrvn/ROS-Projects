#!/bin/bash

echo "Checking your solution ..."

sleep 1

roslaunch robot_sim robot_sim.launch &
sleep 3
rosrun robot_sim robot_gui.py &
sleep 1
rosrun robot_sim fake_robot.py &

#Change this value if you need more time for training
sleep 10m

xterm -hold -e "rosrun robot_sim executive.py" 

sleep 1

killall -9 rosmaster
killall python



