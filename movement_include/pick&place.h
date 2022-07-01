#pragma once
#include "gazebo_ros_link_attacher/Attach.h"
#include "ros/ros.h"
#include "move.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Valori posizione standard 
Vector3f STND_POS,STND_ANGLE;
// Clean 
void cleanTh(Eigen::MatrixXf &Th)
{
    for (int i = 0; i < Th.rows(); i++) for (int j = 0; j < Th.cols(); j++) Th(i,j)= 0;
}

// Funzioni gripper
// Close
void  closeGripper(ros::Publisher gripper,float size)
{
    std_msgs::Float64 temp;
    size = size;
    temp.data = size;
    gripper.publish(temp);
};
// Open
void  openGripper(ros::Publisher gripper)
{
    std_msgs::Float64 temp;
    temp.data = 0.5;
    gripper.publish(temp);
};

void take(ros::ServiceClient attach, ros::Publisher ur5_pub[], Eigen::Vector3f vf, Eigen::Vector3f phiF, Eigen::MatrixXf &Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate, ros::Publisher gripper, float gripperValue)
{

    STND_POS << 0, 0.3203, 0.6147;
    STND_ANGLE << -0.4280, -0.0028, 3.0650;

    gazebo_ros_link_attacher::Attach srv;
    VectorXf vv(6);

    Vector3f above_step;
    above_step=vf;
    above_step[2] = above_step[2]+0.5;
    movement(ur5_pub, above_step, phiF, Th, initial_pos, u, loop_rate);
    int rows = Th.rows() - 1;
    for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
    cleanTh(Th);

    movement(ur5_pub, vf, phiF, Th, vv, u, loop_rate);

    // cout<<"take: "<<(blockName)<<std::endl;    // Per mock test commento il link dinamico

    sleep(3);
    srv.request.model_name_1 = "ur5";
    srv.request.link_name_1 = "hand_link";
    srv.request.model_name_2 = blockName;
    srv.request.link_name_2 = "link";
    attach.call(srv);
    closeGripper(gripper,gripperValue);
    // Ho accesso a Th
    rows = Th.rows() - 1;
    for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);

    cleanTh(Th);
    movement(ur5_pub, STND_POS, STND_ANGLE, Th, vv, u, loop_rate);
};

void place(ros::ServiceClient detach, ros::Publisher ur5_pub[], Eigen::Vector3f vf, Eigen::Vector3f phiF, Eigen::MatrixXf &Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate,int blockNumber,ros::Publisher gripper)
{
    STND_POS << 0, 0.3203, 0.6147;
    STND_ANGLE << -0.4280, -0.0028, 3.0650;

    gazebo_ros_link_attacher::Attach srv;
    movement(ur5_pub, vf, phiF, Th, initial_pos, u, loop_rate);

    int rows = Th.rows() - 1;
    VectorXf vv(6);
    for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
    std::cout<<"vv place1:  "<<vv<<std::endl;
    // for (int i = 0; i < 6; i++) v[i] = Th(rows, i + 1);

    cleanTh(Th);
    sleep(1.5);
    Vector3f ffangle;
    ffangle<<u.legoAngle[blockNumber][0],u.legoAngle[blockNumber][1],u.legoAngle[blockNumber][2];
    movement(ur5_pub, vf, ffangle , Th, vv, u, loop_rate);
    sleep(1.5);
    openGripper(gripper);

    srv.request.model_name_1 = "ur5";
    srv.request.model_name_1 = "ur5";
    srv.request.link_name_1 = "hand_link";
    srv.request.model_name_2 = blockName; 
    srv.request.link_name_2 = "link";
    detach.call(srv);

    rows = Th.rows() - 1;
    for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
    std::cout<<"vv place2:  "<<vv<<std::endl;

    cleanTh(Th);
    movement(ur5_pub, STND_POS, STND_ANGLE, Th, vv, u, loop_rate);
}

void take_and_place(ros::ServiceClient attach, ros::ServiceClient detach, ros::Publisher ur5_pub[], Eigen::Vector3f vf1, Eigen::Vector3f vf2, Eigen::Vector3f phiF, Eigen::MatrixXf Th, Eigen::VectorXf initial_pos, char *blockName,int blockNumber, robot::ur5 u, ros::Rate loop_rate,ros::Publisher gripper,float gripperValue)
{
    STND_POS << 0, 0.3203, 0.6147;
    take(attach, ur5_pub, vf1, phiF, Th, initial_pos, blockName, u, loop_rate,gripper, gripperValue);

    int rows = Th.rows() - 1;
    VectorXf v(6);
    for (int i = 0; i < 6; i++) v[i] = Th(rows, i + 1);

    place(detach, ur5_pub, vf2, phiF, Th, v, blockName, u, loop_rate, blockNumber,gripper);
    cleanTh(Th);
    };
