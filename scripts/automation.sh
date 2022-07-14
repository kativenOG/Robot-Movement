#!/bin/bash
allX=(-0.43 -0.344 -0.258 -0.172 -0.086 0 0.086 0.172 0.258 0.344 0.43)
allY=(-0.2 -0.255 -0.31	-0.365	-0.42	-0.475	-0.53	-0.585	-0.64	-0.695	-0.75)
angleArray=(0 0.393 0.785 1.178 1.571 1.963 2.356 2.749 3.141 3.534 3.927 4.32 4.712 5.105 5.498 5.89)

maxAngle=0 
# b=0
for x in ${allY[@]}; do
  for y in ${allX[@]}; do
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

  a=0
  while test ${a} -lt ${maxAngle}  
  do
    # echo ${angleArray[$a]} 
    ( roslaunch --disable-title --no-summary ultimate_robot automation.launch ) & 
    sleep 5
    ( rosrun ultimate_vision automation.py $1 ${angleArray[$a]} ) & 
    ( ../spawners/spawner_automation.py $1 ${x} ${y} ${angleArray[$a]} ) &  
    sleep 9
    ( pkill gzserver )
    ( pkill gzclient )
    ( pkill roslaunch )        
    sleep 15
    (( a++ )) 
  done
  # (( b++ ))
  done
done
# echo $b

