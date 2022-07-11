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
    STND_ANGLE << 0, 3.14, 0;

    gazebo_ros_link_attacher::Attach srv;
    VectorXf vv(6);

    Vector3f above_step;
    above_step=vf;
    above_step[2] = above_step[2]+0.1;
    movement(ur5_pub, above_step, phiF, Th, initial_pos, u, loop_rate);
    int rows = Th.rows() - 1;
    for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
    cleanTh(Th);
    //-------------------------
    /*movement(ur5_pub, vf, phiF, Th, vv, u, loop_rate);
    rows = Th.rows() - 1;
    for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
    cleanTh(Th);
    int y=1;
    float x;
    phiF(0)=0;
    phiF(1)=3.14;
    phiF(2)=0;
    while(y==1){
        std::cout << "pos?";
        std::cin >> y;
        if(y==1){
            std::cout << "posx=";
            std::cin >> x;
            vf(0)=x;
            std::cout << "posy=";
            std::cin >> x;
            vf(1)=x;
            std::cout << "posz=";
            std::cin >> x;
            vf(2)=x;
        }
        std::cout << "ang?";
        std::cin >> y;
        if(y==1){
            std::cout << "ang1=";
            std::cin >> x;
            phiF(0)=x;
            std::cout << "ang2=";
            std::cin >> x;
            phiF(1)=x;
            std::cout << "ang3=";
            std::cin >> x;
            phiF(2)=x;
        }
        movement(ur5_pub, vf, phiF, Th, vv, u, loop_rate);
        rows = Th.rows() - 1;
        for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
        cleanTh(Th);
        std::cout << "grip?";
        std::cin >> y;
        while(y==1){
            std::cout << "gripper=";
            std::cin >> gripperValue;
            closeGripper(gripper,gripperValue);
            std::cout << "new gripper?";
            std::cin >> y;
        }
        std::cout << "vf=" << vf << "\n";
        std::cout << "phief=" << phiF << "\n";
        std::cout << "continuare?";
        std::cin >> y;
    }*/
    //-------------------------
    movement(ur5_pub, vf, phiF, Th, vv, u, loop_rate);

    // cout<<"take: "<<(blockName)<<std::endl;    // Per mock test commento il link dinamico

    sleep(1.5);
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
    movement(ur5_pub, above_step, phiF, Th, vv, u, loop_rate);
};

void place(ros::ServiceClient detach, ros::Publisher ur5_pub[], Eigen::Vector3f vf, Eigen::Vector3f phiF, Eigen::MatrixXf &Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate,int blockNumber,ros::Publisher gripper)
{
    STND_POS << 0, 0.3203, 0.6147;
    STND_ANGLE << -0.4280, -0.0028, 3.0650;

    gazebo_ros_link_attacher::Attach srv;

    int rows = Th.rows() - 1;
    VectorXf vv(6);
    
    Vector3f ffangle;
    ffangle<<u.legoAngle[blockNumber][0],u.legoAngle[blockNumber][1],u.legoAngle[blockNumber][2];
    movement(ur5_pub, vf, ffangle , Th, initial_pos, u, loop_rate);
    sleep(2.5);
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

    vf1(2)=vf1(2)-0.002;
    float indx[11] = {-0.43,-0.344,-0.258,-0.172,-0.086,0,0.086,0.172,0.258,0.344,0.43};
    float indy[11] = {-0.2,-0.255,-0.31,-0.365,-0.42,-0.475,-0.53,-0.585,-0.64,-0.695,-0.75};
    float Mx[11][11] = {
        {0.449, 0.45, 0.451, 0.452, 0.453, 0.453, 0.453, 0.453, 0.454, 0.454, 0.455},
        {0.365, 0.366, 0.366, 0.367, 0.368, 0.368, 0.368, 0.368, 0.368, 0.368, 0.368},
        {0.281, 0.282, 0.282, 0.282, 0.282, 0.282, 0.282, 0.282, 0.282, 0.282, 0.282},
        {0.196, 0.196, 0.196, 0.197, 0.196, 0.196, 0.197, 0.196, 0.196, 0.196, 0.196},
        {0.109, 0.11, 0.109, 0.11, 0.11, 0.11, 0.109, 0.11, 0.109, 0.11, 0.11},
        {0.017, 0.019, 0.02, 0.022, 0.022, 0.022, 0.022, 0.023, 0.023, 0.023, 0.023},
        {-0.076, -0.072, -0.07, -0.068, -0.067, -0.066, -0.066, -0.065, -0.065, -0.064, -0.064},
        {-0.166, -0.162, -0.16, -0.157, -0.156, -0.153, -0.154, -0.153, -0.152, -0.152, -0.151},
        {-0.254, -0.252, -0.248, -0.246, -0.245, -0.243, -0.242, -0.241, -0.24, -0.239, -0.238},
        {-0.343, -0.34, -0.337, -0.335, -0.333, -0.331, -0.33, -0.329, -0.328, -0.327, -0.326},
        {-0.43, -0.428, -0.425, -0.423, -0.421, -0.419, -0.418, -0.417, -0.416, -0.415, -0.415}};

    float My[11][11] = {
        {0.184, 0.241, 0.298, 0.354, 0.41, 0.466, 0.522, 0.578, 0.635, 0.691, 0.748},
        {0.187, 0.244, 0.301, 0.357, 0.414, 0.469, 0.525, 0.581, 0.637, 0.692, 0.749},
        {0.191, 0.249, 0.305, 0.361, 0.417, 0.473, 0.529, 0.584, 0.64, 0.694, 0.751},
        {0.198, 0.254, 0.31, 0.368, 0.422, 0.477, 0.532, 0.587, 0.642, 0.697, 0.752},
        {0.208, 0.262, 0.317, 0.373, 0.426, 0.481, 0.536, 0.591, 0.646, 0.7, 0.755},
        {0.217, 0.27, 0.323, 0.378, 0.431, 0.486, 0.54, 0.594, 0.649, 0.704, 0.757},
        {0.222, 0.275, 0.328, 0.382, 0.435, 0.49, 0.543, 0.598, 0.652, 0.707, 0.76},
        {0.224, 0.278, 0.331, 0.385, 0.438, 0.492, 0.546, 0.6, 0.654, 0.709, 0.762},
        {0.224, 0.279, 0.333, 0.388, 0.44, 0.494, 0.548, 0.602, 0.657, 0.711, 0.764},
        {0.225, 0.279, 0.333, 0.389, 0.442, 0.496, 0.55, 0.604, 0.658, 0.713, 0.767},
        {0.225, 0.28, 0.334, 0.39, 0.443, 0.497, 0.551, 0.606, 0.66, 0.715, 0.772}};

    int i,j;
    float minx=-0.43;
    float maxx=0.43;
    float miny=-0.2;
    float maxy=-0.75;
    float X=vf1(0);
    float Y=vf1(1);
    if(X<minx||X>maxx||Y>miny||Y<maxy){
        std::cerr << "posizione invalida\n";
    }
    else{
        std::cout << "vf " << vf1(0) << " " << vf1(1) << "\n";
        for(int c=0; c<10; c++){
            if(X>indx[c]&&X<indx[c+1]){
                i=c;
            }
            if(Y<indy[c]&&Y>indy[c+1]){
                j=c;
            }
        }
        if(Y<-0.695){
            if(X<-0.344){
                vf1(2)+=0.007;
            }
            if(X>0.344){
                vf1(2)+=0.002;
            }
        }
        float dx1,dx2,dy1,dy2;
        dx1=Mx[i][j]+(Mx[i+1][j]-Mx[i][j])*(X-indx[i])/(indx[i+1]-indx[i]);
        dx2=Mx[i][j+1]+(Mx[i+1][j+1]-Mx[i][j+1])*(X-indx[i])/(indx[i+1]-indx[i]);
        vf1(0)=dx1+(dx2-dx1)*(Y-indy[j])/(indy[j+1]-indy[j]);

        dy1=My[i][j]+(My[i][j+1]-My[i][j])*(Y-indy[j])/(indy[j+1]-indy[j]);
        dy2=My[i+1][j]+(My[i+1][j+1]-My[i+1][j])*(Y-indy[j])/(indy[j+1]-indy[j]);
        vf1(1)=dy1+(dy2-dy1)*(X-indx[i])/(indx[i+1]-indx[i]);

        std::cout << "vf corretto " << vf1(0) << " " << vf1(1) << "\n";
    }
    int g;

    take(attach, ur5_pub, vf1, phiF, Th, initial_pos, blockName, u, loop_rate,gripper, gripperValue);

    int rows = Th.rows() - 1;
    VectorXf v(6);
    for (int i = 0; i < 6; i++) v[i] = Th(rows, i + 1);

    place(detach, ur5_pub, vf2, phiF, Th, v, blockName, u, loop_rate, blockNumber,gripper);
    cleanTh(Th);
};