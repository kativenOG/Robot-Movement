#!/usr/bin/python3

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random
import numpy as np 
import os  
home_path = os.environ["HOME"]

# Numero di settori in cui viene diviso il tavolo ( 2^sectors )  
sectors = 2
# Numero di blocchi per settore ( sempre 4 in questa versione mi raccomando non cambiare ) 
blockXarea=4
# Threshold di distanza minima tra i blocchi 
threshold = 0.088
# numero di blocchi sul tavolo per ogni tipo 
blockTypeCounter = [0,0,0,0,0,0,0,0,0,0,0]

x_start = -0.4
y_start = 0.20
x_sector = abs((0.4/sectors)-(x_start/sectors))
y_sector = abs((0.75/sectors)-(y_start/sectors))

# Definisco chiamata a spawner 
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
        positions = []
        for n in range(blockXarea):
            # Generate random position
            posCnt= True 
            if (n==0):
                pos = Pose(Point(random.uniform((x_sector*i)+x_start+0.06,(x_sector*i)+x_sector+x_start-0.06), random.uniform(-((y_sector*j)+y_start+ 0.06),-((y_sector*j)+y_sector+y_start-0.06)),0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
                positions.append(pos)
                print(positions)
            else:
                while posCnt==True:
                    print("Continua a andare")
                    pos = Pose(Point(random.uniform((x_sector*i)+x_start+0.06,(x_sector*i)+x_sector+x_start-0.06), random.uniform(-((y_sector*j)+y_start+ 0.06),-((y_sector*j)+y_sector+y_start-0.06)),0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
                    for k in range(n):
                        print(positions)
                        if np.sqrt((pos.position.x-positions[k].position.x)**2+(pos.position.y-positions[k].position.y)**2) < threshold:
                            print("Non è andata: ",np.sqrt((pos.position.x-positions[k].position.x)**2+(pos.position.y-positions[k].position.y)**2))
                            break
                        if k == n-1:
                            positions.append(pos)
                            posCnt = False

            # Spawner rudimentale per il punto 4 ( sono stanco :[ ) 
            if(n==0):
                brickNumber = 7 
                sdfName = "/model1.sdf"
            elif(n==1):
                brickNumber = 9
                blockTypeCounter[brickNumber]= blockTypeCounter[brickNumber]+1 
                sdfName = "/model%d.sdf" % (blockTypeCounter[brickNumber])
            elif(n==2):
                brickNumber = 5 
                sdfName = "/model2.sdf"
            elif(n==3):
                if(j==0 and i==0):
                    brickNumber = 3 
                    sdfName = "/model4.sdf"
                else:
                    brickNumber = 4 
                    blockTypeCounter[brickNumber]= blockTypeCounter[brickNumber]+1 
                    sdfName = "/model%d.sdf" % (blockTypeCounter[brickNumber])
            brick= blocks[brickNumber]

            # Passo i dati allo spawner
            spawn_model_client(model_name=''+str(brick)+'_'+str(i)+'_'+str(j), 
            model_xml=open(home_path+'/progetto_ws/src/ultimate_gazebo/models/'+brick+sdfName, 'r').read(),
            robot_namespace='/foo',
            initial_pose=pos,
            reference_frame='world')
