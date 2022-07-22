#!/bin/bash

gnome-terminal -- roslaunch ultimate_robot robot.launch 
sleep 5 
# sceglie il punto in base all'input
if test $1 -eq 1        
then
  gnome-terminal -- ../spawners/spawner_punto1.py   
  gnome-terminal -- rosrun robot_movement point_2_node  
  sleep 8 
  gnome-terminal -- rosrun ultimate_vision ultimate_position.py   
elif test $1 -eq 2        
then
  gnome-terminal -- ../spawners/spawner_punto2.py  
  gnome-terminal -- rosrun robot_movement point_2_node 
  sleep 8 
  gnome-terminal -- rosrun ultimate_vision ultimate_position.py  
elif test $1 -eq 3        
then
  gnome-terminal -- ../spawners/spawner_punto3.py 
  gnome-terminal -- rosrun robot_movement point_3_node 
  sleep 8 
  gnome-terminal -- rosrun ultimate_vision ultimate_position.py  
elif test $1 -eq 4        
then
  gnome-terminal -- ../spawners/spawner_punto4.py  
  gnome-terminal -- rosrun robot_movement point_4_node 
  sleep 8 
  gnome-terminal -- rosrun ultimate_vision ultimate_position_point4.py  
fi 
