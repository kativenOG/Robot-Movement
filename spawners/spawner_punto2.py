#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random

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

already_called = [11,11,11,11,11,11,11,11,11,11,11]
positions = [[-0.33,0.25],[-0.1,0.35],[0.1,0.25],[0.25,0.23],[0.35,0.45],[-0.33,0.5],[-0.1,0.55],[0.1,0.52],[0.25,0.6],[0.35,0.65],[0,0.7]] 

for i in range(0,11):
    # Posizione del blocco 
    pos = Pose(Point(-positions[i][0],-positions[i][1] ,0.777), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))

    # Calcolo tipo del blocco 
    itsNew = False 
    while(itsNew== False):
        brickNumber = random.randint(0,10)
        itsNew = True 
        if (already_called[0]!=11):
            for x in range(0,11) :
                if(already_called[x] == brickNumber):
                    itsNew= False
    # Riempie array blocchi presenti e cerca il vero nome del blocco 
    already_called[i] = brickNumber
    if(brickNumber==7):
        print("Brick 7 Pos: ",pos)
    brick= blocks[brickNumber]

    # Spawn del blocco :) 
    spawn_model_client(model_name=''+str(brick), 
    model_xml=open('../../ultimate_gazebo/models/'+brick+'/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=pos,
    reference_frame='world')
 

