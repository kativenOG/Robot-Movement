#!/usr/bin/python3

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random
import numpy as np 
# Numero di settori in cui viene diviso il tavolo 
sectors = 2
# Numero di blocchi per settore 
blockXarea=3
# ultimo blocchi inseriti nel settore
last_blocks = []
for i in range(blockXarea):
    last_blocks.append(11) 
print(last_blocks)

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


            #Controllo che il blocco che andrei a spawnare sia unico all'interno della sua area
            while True:
                x = False
                brickNumber = random.randint(0,10)
                # if(n==0):
                #     x = True
                for f in range(blockXarea):
                    if(brickNumber == last_blocks[f]):
                        x = True
                if(x==False):
                    break;
            blockTypeCounter[brickNumber]= blockTypeCounter[brickNumber]+1 
            last_blocks[n]= brickNumber
            brick= blocks[brickNumber]
            sdfName = "/model%d.sdf" % (blockTypeCounter[brickNumber])
            # print("Area: ",i," ",j,"  Block: ",brick)
            # Passo i dati allo spawner
            spawn_model_client(model_name=''+str(brick)+'_'+str(i)+'_'+str(j), 
            model_xml=open('../../ultimate_gazebo/models/'+brick+sdfName, 'r').read(),
            robot_namespace='/foo',
            initial_pose=pos,
            reference_frame='world')
        # Resetta i blocchi
        for l in range(blockXarea):
            last_blocks[l]=11 
