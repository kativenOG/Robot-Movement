#!/bin/bash

( roslaunch ultimate_robot robot.launch ) &
sleep 3 
# sceglie il punto in base all'input
if test $1 -eq 1        
then
  ( ../spawners/spawner_punto1.py ) & 
  ( rosrun robot_movement point_2_node ) &
  sleep 4 
  ( rosrun ultimate_vision ultimate_position.py  ) &
elif test $1 -eq 2        
then
  ( ../spawners/spawner_punto2.py ) & 
  ( rosrun robot_movement point_2_node ) &
  sleep 4 
  ( rosrun ultimate_vision ultimate_position.py  ) &
elif test $1 -eq 3        
then
  ( ../spawners/spawner_punto3.py ) & 
  ( rosrun robot_movement point_3_node ) &
  sleep 4 
  ( rosrun ultimate_vision ultimate_position.py  ) &
elif test $1 -eq 4        
then
  ( ../spawners/spawner_punto4.py ) & 
  ( rosrun robot_movement point_4_node ) &
  sleep 4 
  ( rosrun ultimate_vision ultimate_position_point4.py  ) &
fi 
