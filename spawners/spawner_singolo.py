#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random
import numpy as np 
import os  
import sys
home_path = os.environ["HOME"]
# Numero di settori in cui viene diviso il tavolo 
sectors = 2
# Numero di blocchi per settore 

#rpy to Quaternion
def get_quaternion_from_euler(roll, pitch, yaw):
   qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
   qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
   qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
   qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
   return [qx, qy, qz, qw]


# ultimo blocchi inseriti nel settore
# print(last_blocks)

# numero di blocchi sul tavolo per ogni tipo 
blockTypeCounter = [0,0,0,0,0,0,0,0,0,0,0]
# Threshold di distanza minima tra i blocchi 
threshold = 0.12

x_start = -0.4
y_start = 0.20
x_sector = abs((0.4/sectors)-(x_start/sectors))
y_sector = abs((0.75/sectors)-(y_start/sectors))
# (Definisco chiamata a spawner) 
# Call rospy spawn function to spawn objects in gazebo
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

blocks = [
    "X1-Y1-Z2",
    "X1-Y2-Z1",
    "X1-Y2-Z2",
    "X1-Y2-Z2-CHAMFER",
    "X1-Y2-Z2-TWINFILLET",
    "X1-Y3-Z2",
    "X1-Y3-Z2-FILLET",
    "X1-Y4-Z1",
    "X1-Y4-Z2",
    "X2-Y2-Z2",
    "X2-Y2-Z2-FILLET"]

for i in range(sectors):
    for j in range(sectors):
            #Controllo che il blocco che andrei a spawnare sia unico all'interno della sua area
            pos = Pose(Point(0,0,0),Quaternion(0,0,0,0))
            brickNumber = int(sys.argv[1])
            posCnt= True 
            if(j==0 and i==0):
                if(brickNumber==3 or brickNumber==6 or brickNumber==10): #dritto, di lato con punta verso l'alto 
                    pos = Pose(Point(random.uniform((x_sector*i)+x_start+0.06,(x_sector*i)+x_sector+x_start-0.06), random.uniform(-((y_sector*j)+y_start+ 0.06),-((y_sector*j)+y_sector+y_start-0.06)),0.85), Quaternion(0.001809,0.70710,-0.70706,0.00748))
            elif(j==0 and i==1):
                pos = Pose(Point(random.uniform((x_sector*i)+x_start+0.06,(x_sector*i)+x_sector+x_start-0.06), random.uniform(-((y_sector*j)+y_start+ 0.06),-((y_sector*j)+y_sector+y_start-0.06)),0.845), Quaternion(-0.054039,-0.998512,-0.00725,-0.00044))
            elif(j==1 and i==0):
                [xq,yq,zq,wq] = get_quaternion_from_euler(int(sys.argv[2]),int(sys.argv[3]),int(sys.argv[4]))
                pos = Pose(Point(random.uniform((x_sector*i)+x_start+0.06,(x_sector*i)+x_sector+x_start-0.06), random.uniform(-((y_sector*j)+y_start+ 0.06),-((y_sector*j)+y_sector+y_start-0.06)),0.845), Quaternion(xq,yq,zq,wq))
            else:
                break;

            blockTypeCounter[brickNumber]= blockTypeCounter[brickNumber]+1 
            brick= blocks[brickNumber]
            sdfName = "/model%d.sdf" % (blockTypeCounter[brickNumber])
            # print("Area: ",i," ",j,"  Block: ",brick)
            # Passo i dati allo spawner
            spawn_model_client(model_name=''+str(brick)+'_'+str(i)+'_'+str(j), 
            model_xml=open(home_path+'/progetto_ws/src/ultimate_gazebo/models/'+brick+sdfName, 'r').read(),
            robot_namespace='/foo',
            initial_pose=pos,
            reference_frame='world')
