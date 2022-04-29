#include<iostream>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "ur5.h"

using namespace Eigen;
using namespace std;
using namespace robot;

//vector<control_msgs::JointControllerState::ConstPtr> initial_jnt_pos;
RowVectorXf initial_jnt_pos(7);
ur5 u;

void shoulder_pan_getter(const control_msgs::JointControllerState::ConstPtr& val){
    initial_jnt_pos[0]= val->process_value;
}
void shoulder_lift_getter(const control_msgs::JointControllerState::ConstPtr& val){
    initial_jnt_pos[1]=val->process_value;
}
void elbow_getter(const control_msgs::JointControllerState::ConstPtr& val){
    initial_jnt_pos[2]=val->process_value;
}
void wrist_1_getter(const control_msgs::JointControllerState::ConstPtr& val){
    initial_jnt_pos[3]=val->process_value;
}
void wrist_2_getter(const control_msgs::JointControllerState::ConstPtr& val){
    initial_jnt_pos[4]=val->process_value;
}
void wrist_3_getter(const control_msgs::JointControllerState::ConstPtr& val){
    initial_jnt_pos[5]=val->process_value;
}
void gripper_getter(const control_msgs::JointControllerState::ConstPtr& val){
    initial_jnt_pos[6]=val->process_value;
}


int main(int argc,char ** argv){

    // Creo il nodo 
    ros::init(argc,argv,"gatto_node");
    // Creo un nodo interno al nodo principale
    ros::NodeHandle n;
    // Creo un array per il publishing, ha una casella per ogni joint
    ros::Publisher ur5_joint_array_pub[7];

    // lego l'array ai vari topic, in cui pubblicheranno i valori di posizione      
    ur5_joint_array_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command",1000);
    ur5_joint_array_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command",1000);
    ur5_joint_array_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command",1000);
    ur5_joint_array_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command",1000);
    ur5_joint_array_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command",1000);
    ur5_joint_array_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command",1000);
    ur5_joint_array_pub[6] = n.advertise<std_msgs::Float64>("/gripper_controller/command",1000);

    // creo subscriber che ascoltano nei topic di poszione dei joint, e si salvano la loro poszione nello spazio tramite dei wrapper :) 
    ros::Subscriber shoulder_pan_joint_sub = n.subscribe("",1000,shoulder_pan_getter); 
    ros::Subscriber shoulder_lift_joint_sub = n.subscribe("",1000,shoulder_lift_getter); 
    ros::Subscriber elbow_joint_sub = n.subscribe("",1000,elbow_getter); 
    ros::Subscriber wrist_1_joint_sub = n.subscribe("",1000,wrist_1_getter); 
    ros::Subscriber wrist_2_joint_sub = n.subscribe("",1000,wrist_2_getter); 
    ros::Subscriber wrist_3_joint_sub = n.subscribe("",1000,wrist_3_getter); 
    ros::Subscriber left_knucle_joint_sub = n.subscribe("",1000,gripper_getter); 

    ros::spinOnce();
    
    
    Vector3f v1;
    v1 << 0.5, 0.5, 0.5;
    Vector3f v2;
    v2 << M_PI / 4, M_PI / 4, M_PI / 4;


    // Matrici dell'output del p2pMotionPlan
    MatrixXf Th;
    MatrixXf xE;
    MatrixXf phiE;
    RowVectorXf appo = initial_jnt_pos.block(0,0,1,6);

    u.p2pMotionPlan(appo ,v1 , v2, 0.1 ,Th, xE, phiE);
    for(int i=0;i<Th.rows();i++)
    for(int j=1; j<7;j++){
        ur5_joint_array_pub[j-1]=Th(i,j);
    }     



    return 0;
}


