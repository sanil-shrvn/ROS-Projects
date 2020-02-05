#!/bin/bash

echo "Checking your solution ..."

sleep 3

roslaunch robot_sim robot_sim.launch &
sleep 10
rosrun robot_sim learn_dqn.py &

#Change this value if you need more time for training
sleep 120

rosrun robot_sim cartpole_gui.py &

sleep 1

xterm -hold -e "rosrun robot_sim executive.py" 

sleep 1

killall -9 rosmaster
killall python



