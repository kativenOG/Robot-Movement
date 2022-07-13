#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random
import sys 
import math 


def ToQuaternion(yaw, pitch, roll): #  yaw (Z), pitch (Y), roll (X)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = (cr * cp * cy) + (sr * sp * sy)
    x = (sr * cp * cy) - (cr * sp * sy)
    y = (cr * sp * cy) + (sr * cp * sy)
    z = (cr * cp * sy) - (sr * sp * cy)
    q = Quaternion(w,x,y,z)
    return q;

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

type_arg=int(sys.argv[1])
x_arg = float(sys.argv[2])
y_arg = float(sys.argv[3])
yaw_arg=float(sys.argv[4])

#Generate random position
q=ToQuaternion(0,3.14,yaw_arg)
pos = Pose(Point( x_arg,y_arg,0.785), q)
#Get a random lego block from all legos
brick=blocks[type_arg]
#Call rospy spawn function to spawn objects in gazebo
#spawna direttamente in gazebo !
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(model_name=str(brick), 
    model_xml=open('../../ultimate_gazebo/models/'+brick+'/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=pos,
    reference_frame='world')
 
