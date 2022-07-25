#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random
import numpy as np 
import os  
home_path = os.environ["HOME"]

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

listb = [10,9,5,7,10,3,10,5,9,10,1,8]
listx = [-0.3227160986680035,-0.19177166804465817,-0.092605,0.15748172953976775,0.32980653721293085,0.1373761183408708,-0.17466905112219774,-0.2737620502687227,-0.0677530272435578,0.2941591559002042,0.26280884845502767,0.1470698189798296]

listy = [-0.3794315900537426,-0.3847538480716189,-0.312364,-0.26803277730564945,-0.30507755278993426,-0.40215201463465156,-0.6568218403152696,-0.5364349593740736,-0.5503671834896477,-0.6629870305938496,-0.5459389371160807,-0.6028312896337975]

qx = [-0.0029306255749322954,5.748062633364797e-06,0.054040714552956164,8.799605449855962e-08,-0.002993867308130692,2.0843839626185233e-06,1.9827958554906826e-06,1.9827958554906826e-06,1.9827958554906826e-06,-2.3401441073011895e-06,8.799605449855962e-08,-7.96687701186931e-07]
qy = [0.707102019827467,2.827513531422586e-06,0.99853752872772,-4.3164246532577244e-06,0.7071018430249919,1.7406659573028725e-07,2.578702299797703e-06,2.578702299797703e-06,2.578702299797703e-06,1.513443569517861e-06,-4.3164246532577244e-06,-5.936228376115613e-06]
qz = [-0.70710015194385,0.999606447034842,0.001550478641763734,-0.4021317869465948,-0.7071003062479622,0.97869681179992,-0.8735432989090897,-0.8735432989090897,-0.8735432989090897,-0.928504561043908,-0.4021317869465948,0.9111247530185828]
qw = [0.0027422820132607236,0.028052647029195205,3.014839112391267e-05,0.9155817964051596,0.0026790396113661906,0.20531086324960302,0.48674644828744623,0.48674644828744623,0.48674644828744623,0.3713209933640878,0.9155817964051596,0.4121306642327808]

counter=0 
blockTypeCounter = [0,0,0,0,0,0,0,0,0,0,0]
for j in range(2):
    for i in range(2):
        for x in range(3): 
            brickNumber= listb[counter]
            brick= blocks[brickNumber]
    
            pos = Pose(Point(listx[counter],listy[counter],0.855), Quaternion(qx[counter],qy[counter],qz[counter],qw[counter]))

            blockTypeCounter[brickNumber]= blockTypeCounter[brickNumber]+1 
            sdfName = "/model%d.sdf" % (blockTypeCounter[brickNumber])

            spawn_model_client(model_name=''+str(brick)+'_'+str(i)+'_'+str(j), 
            model_xml=open(home_path+'/progetto_ws/src/ultimate_gazebo/models/'+brick+sdfName, 'r').read(),
            robot_namespace='/foo',
            initial_pose=pos,
            reference_frame='world')

            counter= counter + 1


# NAME =  X2-Y2-Z2-FILLET_1_1
# header: 
#   seq: 1
#   stamp: 
#     secs: 1528
#     nsecs: 464000000
#   frame_id: ''
# pose: 
#   position: 
#     x: 0.2941591559002042
#     y: -0.6629870305938496
#     z: 0.8185002466349397
#   orientation: 
#     x: -2.3401441073011895e-06
#     y: 1.513443569517861e-06
#     z: -0.928504561043908
#     w: 0.3713209933640878
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  0.2975383740241099
# Corretta Y -->  -0.6665193510169196
# NuovaX --->  0.2941591559002042
# NuovaY --->  -0.6629870305938496
# NAME =  X1-Y4-Z2_1_1
# header: 
#   seq: 1
#   stamp: 
#     secs: 1528
#     nsecs: 981000000
#   frame_id: ''
# pose: 
#   position: 
#     x: 0.1470698189798296
#     y: -0.6028312896337975
#     z: 0.8185007525564079
#   orientation: 
#     x: -7.96687701186931e-07
#     y: -5.936228376115613e-06
#     z: 0.9111247530185828
#     w: 0.4121306642327808
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  0.15015089038035176
# Corretta Y -->  -0.6043409245666018
# NuovaX --->  0.1470698189798296
# NuovaY --->  -0.6028312896337975
# NAME =  X1-Y3-Z2_0_1
# header: 
#   seq: 1
#   stamp: 
#     secs: 1529
#     nsecs: 492000000
#   frame_id: ''
# pose: 
#   position: 
#     x: -0.2737620502687227
#     y: -0.5364349593740736
#     z: 0.8184904424273387
#   orientation: 
#     x: -1.0417632500988994e-06
#     y: -4.4724460009913037e-07
#     z: 0.46697166015909397
#     w: 0.8842722819397736
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  -0.27778907671588615
# Corretta Y -->  -0.5369396668970904
# NuovaX --->  -0.2737620502687227
# NuovaY --->  -0.5364349593740736
# NAME =  X2-Y2-Z2_0_0
# header: 
#   seq: 1
#   stamp: 
#     secs: 1530
#     nsecs:   3000000
#   frame_id: ''
# pose: 
#   position: 
#     x: -0.19177166804465817
#     y: -0.3847538480716189
#     z: 0.8184907162807143
#   orientation: 
#     x: 5.748062633364797e-06
#     y: 2.827513531422586e-06
#     z: 0.999606447034842
#     w: 0.028052647029195205
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  -0.19429935150748853
# Corretta Y -->  -0.383041165582276
# NuovaX --->  -0.19177166804465817
# NuovaY --->  -0.3847538480716189
# NAME =  X2-Y2-Z2-FILLET_0_0
# header: 
#   seq: 1
#   stamp: 
#     secs: 1530
#     nsecs: 512000000
#   frame_id: ''
# pose: 
#   position: 
#     x: -0.3227160986680035
#     y: -0.3794315900537426
#     z: 0.8214986126696535
#   orientation: 
#     x: -0.0029306255749322954
#     y: 0.707102019827467
#     z: -0.70710015194385
#     w: 0.0027422820132607236
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  -0.331882598925076
# Corretta Y -->  -0.3738449668523614
# NuovaX --->  -0.3227160986680035
# NuovaY --->  -0.3794315900537426
# YAWINT:  0.0
# NAME =  X2-Y2-Z2-FILLET_1_0
# header: 
#   seq: 1
#   stamp: 
#     secs: 1531
#     nsecs:  19000000
#   frame_id: ''
# pose: 
#   position: 
#     x: 0.32980653721293085
#     y: -0.30507755278993426
#     z: 0.8215042474880871
#   orientation: 
#     x: -0.002993867308130692
#     y: 0.7071018430249919
#     z: -0.7071003062479622
#     w: 0.0026790396113661906
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  0.3362493110353317
# Corretta Y -->  -0.2980603036604425
# NuovaX --->  0.32980653721293085
# NuovaY --->  -0.30507755278993426
# YAWINT:  0.0
# NAME =  X1-Y2-Z2_1_0
# header: 
#   seq: 1
#   stamp: 
#     secs: 1531
#     nsecs: 530000000
#   frame_id: ''
# pose: 
#   position: 
#     x: 0.15748172953976775
#     y: -0.26803277730564945
#     z: 0.8054904056853615
#   orientation: 
#     x: 0.035601931382447896
#     y: 0.7062102840499648
#     z: -0.03559694300126961
# w: 0.7062098801580674
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  0.1571511167238026
# Corretta Y -->  -0.2673135869168917
# NuovaX --->  0.15748172953976775
# NuovaY --->  -0.26803277730564945
# YAWINT:  1.4840579690393427
# NAME =  X2-Y2-Z2-FILLET_0_1
# header: 
#   seq: 1
#   stamp: 
#     secs: 1532
#     nsecs:  40000000
#   frame_id: ''
# pose: 
#   position: 
#     x: -0.17466905112219774
#     y: -0.6568218403152696
#     z: 0.8184906102746993
#   orientation: 
#     x: 4.4875167404408195e-07
#     y: -2.907780636977854e-06
#     z: -0.9877396455982148
#     w: 0.15611019346876906
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  -0.17699120416091205
# Corretta Y -->  -0.6594838067088757
# NuovaX --->  -0.17466905112219774
# NuovaY --->  -0.6568218403152696
# NAME =  X2-Y2-Z2_0_1
# header: 
#   seq: 1
#   stamp: 
#     secs: 1532
#     nsecs: 550000000
#   frame_id: ''
# pose: 
#   position: 
#     x: -0.0677530272435578
#     y: -0.5503671834896477
#     z: 0.8184905552191942
#   orientation: 
#     x: 1.9827958554906826e-06
#     y: 2.578702299797703e-06
#     z: -0.8735432989090897
#     w: 0.48674644828744623
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  -0.06745547744225255
# Corretta Y -->  -0.5520922411507638
# NuovaX --->  -0.0677530272435578
# NuovaY --->  -0.5503671834896477
# NAME =  X1-Y3-Z2_0_0
# header: 
#   seq: 1
#   stamp: 
#     secs: 1533
#     nsecs:  55000000
#   frame_id: ''
# pose: 
#   position: 
#     x: -0.09260455946448896
#     y: -0.31236387138604876
#     z: 0.8186180022239088
#   orientation: 
#     x: 0.054040714552956164
#     y: 0.99853752872772
#     z: 0.001550478641763734
#     w: 3.014839112391267e-05
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  -0.09430915483386086
# Corretta Y -->  -0.30596626868529436
# NuovaX --->  -0.09260455946448896
# NuovaY --->  -0.31236387138604876
# 4
# NAME =  X1-Y2-Z2-CHAMFER_1_0
# header: 
#   seq: 1
#   stamp: 
#     secs: 1533
#     nsecs: 567000000
#   frame_id: ''
# pose: 
#   position: 
#     x: 0.1373761183408708
#     y: -0.40215201463465156
#     z: 0.8184968608176608
#   orientation: 
#     x: 2.0843839626185233e-06
#     y: 1.7406659573028725e-07
#     z: 0.97869681179992
#     w: 0.20531086324960302
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  0.1375690101761401
# Corretta Y -->  -0.40461242234582523
# NuovaX --->  0.1373761183408708
# NuovaY --->  -0.40215201463465156
# NAME =  X1-Y2-Z1_1_1
# header: 
#   seq: 1
#   stamp: 
#     secs: 1534
#     nsecs:  77000000
#   frame_id: ''
# pose: 
#   position: 
#     x: 0.26280884845502767
#     y: -0.5459389371160807
#     z: 0.8089904373459449
#   orientation: 
#     x: 8.799605449855962e-08
#     y: -4.3164246532577244e-06
#     z: -0.4021317869465948
#     w: 0.9155817964051596
# twist: 
#   linear: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular: 
#     x: 0.0
#     y: 0.0
#     z: 0.0
# success: True
# status_message: "GetModelState: got properties"
# Corretta X -->  0.2619646329914491
# Corretta Y -->  -0.5472149969621984
# NuovaX --->  0.26280884845502767
# NuovaY --->  -0.5459389371160807
# 1
# 2
# 3
# 4
# 5
# 6
# 7
# 8
# 9
# 10
# 11
# 12
# Classe, 0   x: 0.1373761183408708
# y: -0.40215201463465156
# z: 0.02850000187754631
# r: 3.14
# p: 0
# y_1: -0.40489183211743396
# type: 3
# gWidth: -0.16
# Classe, 0   x: 0.26280884845502767
# y: -0.5459389371160807
# z: 0.01900000125169754
# r: 3.14
# p: 0
# y_1: -0.8198672576492622
# type: 1
# gWidth: -0.16
# Classe, 2   x: 0.15748172953976775
# y: -0.26803277730564945
# z: 0.015491136349737644
# r: 23.14
# p: 0
# y_1: 4.625650622629136
# type: 2
# gWidth: 0.17
# Classe, 0   x: -0.19177166804465817
# y: -0.3847538480716189
# z: 0.02850000187754631
# r: 3.14
# p: 0
# y_1: 1.5232132281242874
# type: 9
# gWidth: 0.17
# Classe, 0   x: -0.0677530272435578
# y: -0.5503671834896477
# z: 0.02850000187754631
# r: 3.14
# p: 0
# y_1: 1.0303767594355482
# type: 9
# gWidth: 0.17
# Classe, 0   x: 0.1470698189798296
# y: -0.6028312896337975
# z: 0.02850000187754631
# r: 3.14
# p: 0
# y_1: -0.8408966428809639
# type: 8
# gWidth: -0.16
# Classe, 0   x: -0.2737620502687227
# y: -0.5364349593740736
# z: 0.02850000187754631
# r: 3.14
# p: 0
# y_1: 0.9827937939229934
# type: 5
# gWidth: -0.16
# Classe, 1   x: -0.09578867929489061
# y: -0.3442050638540622
# z: 0.017266771793365478
# r: 11.570796326794897
# p: 3.0419239829516322
# y_1: 3.141592653589793
# type: 5
# gWidth: -0.16
# Classe, 0   x: 0.2941591559002042
# y: -0.6629870305938496
# z: 0.02850000187754631
# r: 3.14
# p: 0
# y_1: 0.7853981633974483
# type: 10
# gWidth: 0.17
# Classe, 0   x: -0.17466905112219774
# y: -0.6568218403152696
# z: 0.02850000187754631
# r: 3.14
# p: 0
# y_1: -1.2490457873276737
# type: 10
# gWidth: 0.17
# Classe, 2   x: -0.3227160986680035
# y: -0.3794315900537426
# z: 0.03150367736816406
# r: 21.5707963267949
# p: 3.141592653589793
# y_1: 0
# type: 10
# gWidth: 0.06300000101327896
# Classe, 2   x: 0.32980653721293085
# y: -0.30507755278993426
# z: 0.0315093994140625
# r: 21.5707963267949
# p: 3.141592653589793
# y_1: 0
# type: 10
# gWidth: 0.06300000101327896
#
