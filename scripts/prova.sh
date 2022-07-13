#!/bin/bash
x=0
y=-0.5
a=0
angleArray=(0 0.393 0.785 1.178 1.571 1.963 2.356 2.749 3.141 3.534 3.927 4.32 4.712 5.105 5.498 5.89)
       
if test $1 -eq 0 
then
  maxAngle=4
elif test $1 -eq 9  
then 
  maxAngle=4 
elif test $1 -eq 10  
then
  maxAngle=16
elif test $1 -eq 6   
then 
  maxAngle=16
elif test $1 -eq 3   
then 
  maxAngle=16
else 
  maxAngle=8 
fi

while test ${a} -lt ${maxAngle}  
  do
  # echo ${angleArray[$a]} 
  ( roslaunch --disable-title --no-summary ultimate_robot automation.launch ) & 
  sleep 5
  # ( rosrun ultimate_vision automation.py )
  ( ../spawners/spawner_automation.py $1 ${x} ${y} ${angleArray[$a]} ) &  
  sleep 8
  ( pkill gzserver )
  ( pkill gzclient )
  ( pkill roslaunch )       
  sleep 15
  (( a++ )) 
done
