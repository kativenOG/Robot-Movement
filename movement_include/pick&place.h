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
    // CAMBIATO A FISSO 1 
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
    //STND_ANGLE << -0.4280, -0.0028, 3.0650;
    STND_ANGLE << 0, 3.14, 0;

    gazebo_ros_link_attacher::Attach srv;
    VectorXf vv(6);

    Vector3f above_step;
    above_step=vf;
    //above_step[2] = above_step[2]+0.5;
    above_step[2] = above_step[2]+0.1;
    movement(ur5_pub, above_step, phiF, Th, initial_pos, u, loop_rate);
    int rows = Th.rows() - 1;
    for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
    cleanTh(Th);
    //-------------------------
    movement(ur5_pub, vf, phiF, Th, vv, u, loop_rate);
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
        /*std::cout << "ang?";
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
        }*/
        movement(ur5_pub, vf, phiF, Th, vv, u, loop_rate);
        rows = Th.rows() - 1;
        for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
        cleanTh(Th);
        /*std::cout << "grip?";
        std::cin >> y;
        while(y==1){
            std::cout << "gripper=";
            std::cin >> gripperValue;
            closeGripper(gripper,gripperValue);
            std::cout << "new gripper?";
            std::cin >> y;
        }*/
        std::cout << "vf=" << vf << "\n";
        //----
        closeGripper(gripper,-1);
        //----
        std::cout << "phief=" << phiF << "\n";
        /*std::cout << "continuare?";
        std::cin >> y;*/
    }
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
    //*movement(ur5_pub, vf, phiF, Th, initial_pos, u, loop_rate);

    int rows = Th.rows() - 1;
    VectorXf vv(6);
    //*for (int i = 0; i < 6; i++) vv[i] = Th(rows, i + 1);
    //*std::cout<<"vv place1:  "<<vv<<std::endl;
    // for (int i = 0; i < 6; i++) v[i] = Th(rows, i + 1);

    //*cleanTh(Th);
    //*sleep(1.5);
    Vector3f ffangle;
    ffangle<<u.legoAngle[blockNumber][0],u.legoAngle[blockNumber][1],u.legoAngle[blockNumber][2];
    //movement(ur5_pub, vf, ffangle , Th, vv, u, loop_rate);
    movement(ur5_pub, vf, ffangle , Th, initial_pos, u, loop_rate);
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
    //-----------------------
    /*std::cout << blockNumber << "\n";
    if(blockNumber==0){
        //vf1(0)=vf1(0)+0.024;
        //vf1(1)=vf1(1)+0.011;
        vf1(2)=vf1(2)-0.002;
        float block0x[7][7] = {{-162,-172,-167,-163,-173,-168,-164},
                               {-107,-103,-113,-108,-104,-114,-109},
                               {-38,-48,-44,-40,-54,-45,-41},
                               {15,6,10,15,19,9,13},
                               {70,60,65,69,59,64,68},
                               {128,129,119,124,128,118,123},
                               {194,184,188,192,183,187,191}};
        float block0y[7][7] = {{170,115,47,-8,-63,-118,-186},
                               {174,106,51,-4,-59,-113,-182},
                               {179,110,41,-14,-49,-123,-178},
                               {169,114,45,-10,-64,-119,-188},
                               {173,119,50,-5,-60,-114,-168},
                               {178,109,54,-15,-55,-124,-179},
                               {168,113,44,-11,-64,-120,-189}};
        int i,j;
        float minx=-0.3;
        float maxy=-0.25;
        float X=vf1(0);
        float Y=vf1(1);
        if(X<minx) i=0;
        if(Y>maxy) j=0;
        if(X>0.2) i=5;
        if(Y<-0.75) j=5;
        for(int c=0; c<6; c++){
            if(X>minx+0.1*c&&X<minx+0.1*(c+1)){
                i=c;
            }
            if(Y<maxy-0.1*c&&Y>maxy-0.1*(c+1)){
                j=c;
            }
        }
        float dx,dx1,dx2,dy,dy1,dy2;
        dx1=block0x[i][j]+(block0x[i+1][j]-block0x[i][j])*(X-(minx+0.1*i))/0.1;
        dx2=block0x[i][j+1]+(block0x[i+1][j+1]-block0x[i][j+1])*(X-(minx+0.1*i))/0.1;
        dx=dx1+(dx2-dx1)*(Y-(maxy-0.1*j))/0.1;

        dy1=block0y[i][j]-(block0y[i][j+1]-block0y[i][j])*(Y-(maxy-0.1*j))/0.1;
        dy2=block0y[i+1][j]-(block0y[i+1][j+1]-block0y[i+1][j])*(Y-(maxy-0.1*j))/0.1;
        dy=dy1+(dy2-dy1)*(X-(maxy-0.1*j))/0.1;



        std::cout << "vf " << -vf1(0) << " " << -vf1(1) << "\n";
        vf1(0)=vf1(0)+dx*0.0001;
        vf1(1)=vf1(1)+dy*0.0001;
        std::cout << "vf corretto " << -vf1(0) << " " << -vf1(1) << "\n";
    }
    int g;
    std::cout << "iniziare?";
    std::cin >> g;*/
    //-----------------------
    take(attach, ur5_pub, vf1, phiF, Th, initial_pos, blockName, u, loop_rate,gripper, gripperValue);

    int rows = Th.rows() - 1;
    VectorXf v(6);
    for (int i = 0; i < 6; i++) v[i] = Th(rows, i + 1);

    place(detach, ur5_pub, vf2, phiF, Th, v, blockName, u, loop_rate, blockNumber,gripper);
    cleanTh(Th);
};
