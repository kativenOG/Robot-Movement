#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "robot_movement/customMsg.h"
#include "control_msgs/JointControllerState.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Movement Kinematics 
#include "p2pMotionPlan.h"
#include "ur5.h"
#include "direct.h"
#include "inverse.h"
#include "pick&place.h"
#include "handlers.h"

// Link attacher and movement
#include "gazebo_ros_link_attacher/Attach.h"
#include "move.h"

//#include "pick&place.h"

using namespace Eigen;
using namespace std;
using namespace robot;

ur5 u;

int main(int argc, char **argv)
{

    // Creo il nodo
    ros::init(argc, argv, "gatto_node");
    // Creo un nodo interno al nodo principale
    ros::NodeHandle n;

    // Ottengo la rate per il publishing
    ros::Rate loop_rate(RATE);
    // Creo un array per il publishing, ha una casella per ogni joint
    ros::Publisher ur5_joint_array_pub[6];

    ros::Publisher ur5_gripper_pub; 
    // lego l'array ai vari topic, in cui pubblicheranno i valori di posizione
    ur5_joint_array_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", RATE);
    ur5_joint_array_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", RATE);
    ur5_joint_array_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", RATE);
    ur5_joint_array_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", RATE);
    ur5_joint_array_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", RATE);
    ur5_joint_array_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", RATE);
    
    // Publihser per il gripper, range -0.5 - +0.5
    ur5_gripper_pub = n.advertise<std_msgs::Float64>("/gripper_controller/command", RATE);

    // creo subscriber che ascoltano nei topic di poszione dei joint, e si salvano la loro poszione nello spazio tramite dei wrapper :)
    ros::Subscriber shoulder_pan_joint_sub = n.subscribe("/shoulder_pan_joint_position_controller/state", RATE, shoulder_pan_getter);
    ros::Subscriber shoulder_lift_joint_sub = n.subscribe("/shoulder_lift_joint_position_controller/state", RATE, shoulder_lift_getter);
    ros::Subscriber elbow_joint_sub = n.subscribe("/elbow_joint_position_controller/state", RATE, elbow_getter);
    ros::Subscriber wrist_1_joint_sub = n.subscribe("/wrist_1_joint_position_controller/state", RATE, wrist_1_getter);
    ros::Subscriber wrist_2_joint_sub = n.subscribe("/wrist_2_joint_position_controller/state", RATE, wrist_2_getter);
    ros::Subscriber wrist_3_joint_sub = n.subscribe("/wrist_3_joint_position_controller/state", RATE, wrist_3_getter);
    ros::Subscriber left_knucle_joint_sub = n.subscribe("/gripper_joint_position/state", RATE, gripper_getter); // se è aperto o chiuso (non proprio un angolo )
    ros::Subscriber brick_sub= n.subscribe("/brick", RATE, brick_getter);

    // Indispensabili 
    ros::ServiceClient dynLinkAtt = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    ros::ServiceClient dynLinkDet = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    MatrixXf Th;
    Vector3f vff;
    int blockInt =(int) blockNumber[0];
    vff<<-u.legoPos[blockInt][0],u.legoPos[blockInt][1],0.15;
    
    // prendo solo il primo blocco, quindi il primo valore della matrice 
    MatrixXf ee_pos,ee_angle;
    ee_pos<< block_position(0,0),block_position(0,1),block_position(0,2);
    ee_angle<< block_angle(0,0),block_angle(0,1),block_angle(0,2);

    take_and_place( dynLinkAtt, dynLinkDet , ur5_joint_array_pub , ee_pos , vff, ee_angle , Th, initial_jnt_pos, u.legos[blockInt], u, loop_rate);
    return 0;
}
