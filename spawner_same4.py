#!/usr/bin/python3

#spawner che genera casualmente un solo blocco (usato per il testing con mock_node) scritto da Carlo 
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random
# Numero di settori in cui viene diviso il tavolo 
sectors = 2
x_start = -0.4
y_start = 0.25
# y1 = 0.25/sectors
# y2 = 0.95/sectors
# x1 = -0.4/sectors
# x2 = 0.4/sectors
y_sector = abs((0.95/sectors)-(y_start/sectors))
x_sector = abs((0.4/sectors)-(x_start/sectors))
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
print(x_sector,y_sector)

for i in range(sectors):
    for j in range(sectors):
        # Generate random position
        pos = Pose(Point(random.uniform((x_sector*i)+x_start,(x_sector*i)+x_sector+x_start), random.uniform(-((y_sector*j)+y_start),-((y_sector*j)+y_sector+y_start)),0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
        # brickNumber= i+j
        brickNumber = random.randint(0,10)
        brick=blocks[brickNumber]
        # Passo i dati allo spawner
        spawn_model_client(model_name=''+str(brick)+'_'+str(i)+'_'+str(j), 
            model_xml=open('../ultimate_gazebo/models/lego_'+brick+'/model.sdf', 'r').read(),
            robot_namespace='/foo',
            initial_pose=pos,
            reference_frame='world')
