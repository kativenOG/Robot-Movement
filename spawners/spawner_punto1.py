#!/usr/bin/python3

#spawner che genera casualmente un solo blocco (usato per il testing con mock_node) scritto da Carlo 
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random
import os  
home_path = os.environ["HOME"]

#Array containing all lego blocks names
# blocks  = ["X1-Y2-Z1", "X2-Y2-Z2", "X1-Y3-Z2", "X1-Y2-Z2", "X1-Y2-Z2-CHAMFER", "X1-Y4-Z2", "X1-Y1-Z2", "X1-Y2-Z2-TWINFILLET", "X1-Y3-Z2-FILLET", "X1-Y4-Z1", "X2-Y2-Z2-FILLET"];
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

#Generate random position
pos = Pose(Point(random.uniform(-0.33, 0.33), random.uniform(-0.25, -0.73),0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
#Get a random lego block from all legos
brickNumber = random.randint(0,10)
brick=blocks[brickNumber]
print(pos)
print(brick)
print(brickNumber)
#Call rospy spawn function to spawn objects in gazebo
#spawna direttamente in gazebo !
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(model_name=''+str(brick), 
    model_xml=open(home_path+'/progetto_ws/src/ultimate_gazebo/models/'+brick+'/model1.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=pos,
    reference_frame='world')
