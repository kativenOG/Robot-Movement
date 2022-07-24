#pragma once
#include "gazebo_ros_link_attacher/Attach.h"
#include "ros/ros.h"
#include "move.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <string.h>
#include <cstdio> 
// Valori posizione standard 
Vector3f STND_POS,STND_ANGLE;

// funzione per incollare i blocchi tra di loro 
void attach_detach_blocks(ros::ServiceClient service, char* block1,char* block2){ char link_choice[20]= "link";
    gazebo_ros_link_attacher::Attach srv;

    srv.request.model_name_1 = block1;
    srv.request.link_name_1 = "link";

    srv.request.model_name_2 = block2; 
    if(!strcmp(block2,"end_table")) std::strcpy(link_choice,"top_plate");
    std::cout << "BLOCK1: " <<link_choice<< std::endl;
    srv.request.link_name_2 = link_choice;

    service.call(srv);
}
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
    movement(ur5_pub, vf, phiF, Th, vv, u, loop_rate);

    sleep(1.5);
    srv.request.model_name_1 = "ur5";
    srv.request.link_name_1 = "hand_link";
    srv.request.model_name_2 = blockName;
    srv.request.link_name_2 = "link";
    attach.call(srv);
    closeGripper(gripper,gripperValue);

    rows = Th.rows() - 1;
    for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
    cleanTh(Th);
    movement(ur5_pub, above_step, phiF, Th, vv, u, loop_rate);
};

void place(ros::ServiceClient attach,ros::ServiceClient detach, ros::Publisher ur5_pub[], Eigen::Vector3f vf, Eigen::Vector3f phiF, Eigen::MatrixXf &Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate,int blockNumber,ros::Publisher gripper,int rotType,float oldZ)
{
    STND_POS << 0, 0.3203, 0.6147;
    STND_ANGLE << -0.4280, -0.0028, 3.0650;

    gazebo_ros_link_attacher::Attach srv;


    Vector3f ffangle;
    int counter;
    std::cout << "rotType=" << rotType << "  --------------------------------------------------------\n";   
    if(u.castleMode == false){
        std::cout << " castlemod false ---------------------------------------------------------\n";
        if(rotType==0){ // Dritto
            std::cout << " angol dritte ---------------------------------------------------------\n";
            ffangle<<u.legoAngle[blockNumber][0],u.legoAngle[blockNumber][1],u.legoAngle[blockNumber][2];
        }else if(rotType==1){ //Sotto sopra 
            std::cout << " angolo sotto sopra ---------------------------------------------------------\n";
            if(blockNumber==0||blockNumber==9){
                std::cout << "block " << blockNumber << "-----------------------------------------\n";
                ffangle<<0,-1.57,1.57;
            }else if(blockNumber==2||blockNumber==4||blockNumber==5){
                std::cout << "block " << blockNumber << "-----------------------------------------\n";
                ffangle<<0,3.14,1.57;
            }else if(blockNumber==8){
                std::cout << "block " << blockNumber << "-----------------------------------------\n";
                ffangle<<0,3.92,1.57;
            }else{
                std::cout << "Prese sotto sopra assenti per block=" << blockNumber << "\n";
            }
        }else if(rotType==2){ // Di lato
            std::cout << " angol laterali ---------------------------------------------------------\n";
            if(blockNumber==1 || blockNumber==7){
                std::cout << "Prese laterali assenti per block=" << blockNumber << "\n";
            }else if(blockNumber==2){
                if(oldZ>0.003){
                    ffangle<<u.lSideAngles[1][0],u.lSideAngles[1][1],u.lSideAngles[1][2];
                    std::cout << " pos laterali 1x2 lato corto ---------------------------------------------------------\n";
                }else{
                    ffangle<<u.lSideAngles[blockNumber][0],u.lSideAngles[blockNumber][1],u.lSideAngles[blockNumber][2];
                    std::cout << " pos laterali 1x2 lato lungo ---------------------------------------------------------\n";
                }
            }else{
                ffangle<<u.lSideAngles[blockNumber][0],u.lSideAngles[blockNumber][1],u.lSideAngles[blockNumber][2];
                std::cout << "block " << blockNumber << "-----------------------------------------\n";
            }
        }
    }else{
        std::cout << " castlemod on ---------------------------------------------------------\n";
        if(rotType==0){
          if(blockNumber==4 && u.cTypeThree==3){
              ffangle<<u.castleAngle[1][0],u.castleAngle[1][1],u.castleAngle[1][2]; // cicla tra le 2 posizioni inclinate
          }else if(blockNumber == 7 ){ // y4
              counter = u.cTypeOne%2; 
              ffangle<<u.castleAngle[2+counter][0],u.castleAngle[2+counter][1],u.castleAngle[2+counter][2]; // cicla tra le 2 posizioni inclinate
          }else if(blockNumber = 5){ //y3
              counter = u.cTypeTwo%2; 
              ffangle<<u.castleAngle[counter][0],u.castleAngle[counter][1],u.castleAngle[counter][2]; // cicla tra dritti e in piedi 
          }else ffangle<<u.castleAngle[0][0],u.castleAngle[0][1],u.castleAngle[0][2]; // tutti gli altri blocchi vanno storti di 90 gradi 
        }else if(rotType==2){
            std::cout << " angol laterali ---------------------------------------------------------\n";
        
        }else{
            std::cout << " angolo sotto sopra ---------------------------------------------------------\n";
        }
    }
    int rows;
    VectorXf vv(6);
    Vector3f above_step;
    above_step=vf;
    above_step[2] = above_step[2]+0.25;
    
    Vector3f ffangle2 = ffangle;
    movement(ur5_pub, above_step, ffangle, Th, initial_pos, u, loop_rate);
    rows = Th.rows() - 1;
    for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
    cleanTh(Th);   
        movement(ur5_pub, vf, ffangle2, Th, vv, u, loop_rate);
    sleep(1.8);
    openGripper(gripper);

    // Attacca blocco a tavolo o blocco precendente in colonna 
    attach_detach_blocks(attach,blockName,u.lastLego[blockNumber]);

    // Stacca il blocco dal gripper 
    srv.request.model_name_1 = "ur5";
    srv.request.model_name_1 = "ur5";
    srv.request.link_name_1 = "hand_link";
    srv.request.model_name_2 = blockName; 
    srv.request.link_name_2 = "link";
    detach.call(srv);   

    // Salva l'ultimo blocco attaccato per quella posizione
    std::strcpy(u.lastLego[blockNumber],blockName);

    rows = Th.rows() - 1;
    for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);

    cleanTh(Th);
    movement(ur5_pub, STND_POS, STND_ANGLE, Th, vv, u, loop_rate);
}

void take_place_link(ros::ServiceClient attach, ros::ServiceClient detach, ros::Publisher ur5_pub[], Eigen::Vector3f vf1, Eigen::Vector3f vf2, Eigen::Vector3f phiF, Eigen::MatrixXf Th, Eigen::VectorXf initial_pos, char *blockName,int blockNumber, robot::ur5 u, ros::Rate loop_rate,ros::Publisher gripper,float gripperValue,int rotType)
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

    float Mz[11][11] = {
        {0.1528, 0.1522, 0.1512, 0.1514, 0.1516, 0.1519, 0.1521, 0.1524, 0.1526, 0.1529, 0.153},
        {0.1522, 0.1521, 0.1521, 0.1521, 0.1521, 0.1522, 0.1524, 0.1527, 0.1528, 0.1532, 0.1536},
        {0.1518, 0.1517, 0.152, 0.1522, 0.1524, 0.1527, 0.1528, 0.1532, 0.1536, 0.154, 0.1544},
        {0.1512, 0.1512, 0.1513, 0.1518, 0.1518, 0.1526, 0.1529, 0.1532, 0.1535, 0.1538, 0.1545},
        {0.1506, 0.1505, 0.1505, 0.1507, 0.1511, 0.1517, 0.1525, 0.1528, 0.1533, 0.1538, 0.1544},
        {0.1512, 0.1509, 0.151, 0.1512, 0.1513, 0.1515, 0.1517, 0.1519, 0.1523, 0.1525, 0.1528},
        {0.1506, 0.1504, 0.1504, 0.1506, 0.1507, 0.1508, 0.151, 0.1514, 0.1515, 0.1516, 0.1517},
        {0.1508, 0.1508, 0.1508, 0.1511, 0.1515, 0.1518, 0.1528, 0.1533, 0.1538, 0.1541, 0.155},
        {0.1515, 0.1515, 0.1518, 0.1523, 0.1526, 0.1528, 0.1532, 0.1535, 0.154, 0.1541, 0.1548},
        {0.1525, 0.1524, 0.1527, 0.1528, 0.1531, 0.1533, 0.1536, 0.1538, 0.1542, 0.1544, 0.155},
        {0.154, 0.1538, 0.1538, 0.1538, 0.1541, 0.1541, 0.1542, 0.1543, 0.1546, 0.1552, 0.157}};

    int i,j;
    float minx=-0.43;
    float maxx=0.43;
    float miny=-0.2;
    float maxy=-0.75;
    float X=vf1(0);
    float Y=vf1(1);
    float h=0.057;
    float gap=0.001;
    if(X<minx||X>maxx||Y>miny||Y<maxy){
        std::cerr << "posizione invalida\n";
    }
    else{
        std::cout << "vf " << vf1(0) << " " << vf1(1) << " " << vf1(2) << "\n";
        for(int c=0; c<10; c++){
            if(X>indx[c]&&X<indx[c+1]){
                i=c;
            }
            if(Y<indy[c]&&Y>indy[c+1]){
                j=c;
            }
        }
        float dx1,dx2,dy1,dy2,dz1,dz2;
        dx1=Mx[i][j]+(Mx[i+1][j]-Mx[i][j])*(X-indx[i])/(indx[i+1]-indx[i]);
        dx2=Mx[i][j+1]+(Mx[i+1][j+1]-Mx[i][j+1])*(X-indx[i])/(indx[i+1]-indx[i]);
        vf1(0)=dx1+(dx2-dx1)*(Y-indy[j])/(indy[j+1]-indy[j]);

        dy1=My[i][j]+(My[i][j+1]-My[i][j])*(Y-indy[j])/(indy[j+1]-indy[j]);
        dy2=My[i+1][j]+(My[i+1][j+1]-My[i+1][j])*(Y-indy[j])/(indy[j+1]-indy[j]);
        vf1(1)=dy1+(dy2-dy1)*(X-indx[i])/(indx[i+1]-indx[i]);

        dz1=Mz[i][j]+(Mz[i][j+1]-Mz[i][j])*(Y-indy[j])/(indy[j+1]-indy[j]);
        dz2=Mz[i+1][j]+(Mz[i+1][j+1]-Mz[i+1][j])*(Y-indy[j])/(indy[j+1]-indy[j]);
        vf1(2)+=dz1+(dz2-dz1)*(X-indx[i])/(indx[i+1]-indx[i]) -h +gap;

        

        std::cout << "vf corretto " << vf1(0) << " " << vf1(1) << " " << vf1(2) <<"\n";
    }

    take(attach, ur5_pub, vf1, phiF, Th, initial_pos, blockName, u, loop_rate,gripper, gripperValue);

    int rows = Th.rows() - 1;
    VectorXf v(6);
    for (int i = 0; i < 6; i++) v[i] = Th(rows, i + 1);
    float oldZ= vf1[2];
    place(attach,detach, ur5_pub, vf2, phiF, Th, v, blockName, u, loop_rate, blockNumber,gripper,rotType,oldZ);
    cleanTh(Th);
};
// Parte calibrazione
    //-------------------------
    // int y=1;
    // float x;
    // while(y==1){
    //     std::cout << "pos?";
    //     std::cin >> y;
    //     if(y==1){
    //         std::cout << "posx=";
    //         std::cin >> x;
    //         vf(0)=x;
    //         std::cout << "posy=";
    //         std::cin >> x;
    //         vf(1)=x;
    //         std::cout << "posz=";
    //         std::cin >> x;
    //         vf(2)=x;
    //     }
    //     /*else{
    //         std::cout << "posz?";
    //         std::cin >> y;
    //         while(y==1){
    //             std::cout << "posz=";
    //             std::cin >> x;
    //             vf(2)=x;
    //             movement(ur5_pub, vf, phiF, Th, vv, u, loop_rate);
    //             rows = Th.rows() - 1;
    //             for(int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
    //             cleanTh(Th);
    //             std::cout << "aggiornare?";
    //             std::cin >> y;
    //         }
    //     }*/
    //     std::cout << "ang?";
    //     std::cin >> y;
    //     if(y==1){
    //         std::cout << "ang1=";
    //         std::cin >> x;
    //         phiF(0)=x;
    //         std::cout << "ang2=";
    //         std::cin >> x;
    //         phiF(1)=x;
    //         std::cout << "ang3=";
    //         std::cin >> x;
    //         phiF(2)=x;
    //     }
    //     movement(ur5_pub, vf, phiF, Th, vv, u, loop_rate);
    //     rows = Th.rows() - 1;
    //     for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
    //     cleanTh(Th);
    //     /*std::cout << "grip?";
    //     std::cin >> y;
    //     while(y==1){
    //         std::cout << "gripper=";
    //         std::cin >> gripperValue;
    //         closeGripper(gripper,gripperValue);
    //         std::cout << "new gripper?";
    //         std::cin >> y;
    //     }*/
    //     std::cout << "vf=" << vf << "\n";
    //     std::cout << "phief=" << phiF << "\n";
    //     std::cout << "continuare?";
    //     std::cin >> y;
    // }
    //-------------------------


