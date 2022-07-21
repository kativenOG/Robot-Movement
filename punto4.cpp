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
#include <cstdio>
// Movement Kinematics
#include "p2pMotionPlan.h"
#include "ur5.h"
#include "direct.h"
#include "inverse.h"
#include "handlers.h"

// Link attacher and movement
#include "gazebo_ros_link_attacher/Attach.h"
#include "move.h"
#include "pickPlaceLink.h"

using namespace Eigen;
using namespace std;
using namespace robot;

ur5 u;

int main(int argc, char **argv)
{
    // Creo il nodo
    ros::init(argc, argv, "point_2_node");
    // Creo un nodo interno al nodo principale
    ros::NodeHandle n;

    // Ottengo la rate per il publishing
    ros::Rate loop_rate(RATE);
    // Creo un array per il publishing, ha una casella per ogni joint
    ros::Publisher ur5_joint_array_pub[6];
    ros::Publisher ur5_gripper_pub;
    // lego l'array ai vari topic, in cui pubblicheranno i valori di posizione
    ur5_joint_array_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", QUEUE_SIZE);
    ur5_joint_array_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", QUEUE_SIZE);
    ur5_joint_array_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", QUEUE_SIZE);
    ur5_joint_array_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", QUEUE_SIZE);
    ur5_joint_array_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", QUEUE_SIZE);
    ur5_joint_array_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", QUEUE_SIZE);

    // Publihser per il gripper, range -0.5 - +0.5
    ur5_gripper_pub = n.advertise<std_msgs::Float64>("/gripper_joint_position/command", QUEUE_SIZE);

    // creo subscriber che ascoltano nei topic di poszione dei joint, e si salvano la loro poszione nello spazio tramite dei wrapper :)
    ros::Subscriber shoulder_pan_joint_sub = n.subscribe("/shoulder_pan_joint_position_controller/state", QUEUE_SIZE, shoulder_pan_getter);
    ros::Subscriber shoulder_lift_joint_sub = n.subscribe("/shoulder_lift_joint_position_controller/state", QUEUE_SIZE, shoulder_lift_getter);
    ros::Subscriber elbow_joint_sub = n.subscribe("/elbow_joint_position_controller/state", QUEUE_SIZE, elbow_getter);
    ros::Subscriber wrist_1_joint_sub = n.subscribe("/wrist_1_joint_position_controller/state", QUEUE_SIZE, wrist_1_getter);
    ros::Subscriber wrist_2_joint_sub = n.subscribe("/wrist_2_joint_position_controller/state", QUEUE_SIZE, wrist_2_getter);
    ros::Subscriber wrist_3_joint_sub = n.subscribe("/wrist_3_joint_position_controller/state", QUEUE_SIZE, wrist_3_getter);
    ros::Subscriber left_knucle_joint_sub = n.subscribe("/gripper_joint_position/state", QUEUE_SIZE, gripper_getter); // se è aperto o chiuso (non proprio un angolo )

    // Vision topic with brick Data
    ros::Subscriber kinect_name = n.subscribe("brick", 11, brick_getter);

    // ASYNC SPINNING 
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // METTI SEMPRE COUT PER SINCRONIZZARE COSÌ NON SI PERDONO MESSAGGI
    // int x;
    // cout << "Inizio Programma !";
    // cin >> x;

    // Nodi per il dynamic linker
    ros::ServiceClient dynLinkAtt = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    ros::ServiceClient dynLinkDet = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    // Magari da portare fuori (globali) in pickPlaceLink per permettere di controllare se sono superiori al primo :)    
    // int cTypeOne=0,cTypeTwo=0,cTypeThree=0;

    for (int i = 0; i < 16; i++){
    
        while(cnt==i){
          std::cout << "cnt: " <<cnt<<"  i: "<<i<< std::endl;
        }
    
        // Array posizione finale per il blocco 
        Vector3f vff;
        
        // Calcolo valori o copio tipo blocco, ampiezza gripper e z della posizione finale del prossimo blocco
        Vector3f ee_pos;
        ee_pos<< block_position(i,0),block_position(i,1),block_position(i,2);
        Vector3f ee_angle;
        ee_angle<< block_angle(i,0),block_angle(i,1),block_angle(i,2);
        int blockk = blockNumber[i];
        float gg= gripperWidth[i];
        float fheigth;
        
        // Determina il quadrante del blocco e cambia il nome di conseguenza 
        char blockName[80];
        int ipos,jpos; 
        if(ee_pos[0]< 0) ipos=0;
        else ipos=1;
        if(ee_pos[1]> -0.475) jpos=0; // più vicino all'origine 
        else jpos=1;
        sprintf(blockName,"%s_%i_%i",u.legos[blockk],ipos,jpos);

        char squareBlockNames[4][30];
        u.castleMode = true;

        switch (blockk) {
          case 7: // y4-z1
            fheigth = 0.1586;  // altezza standard blocco z1 + un blocco z2 
            std::strcpy(u.lastLego[7],squareBlockNames[u.cTypeOne]);
            // forse metti un altro link 
            vff <<u.castlePos[5+u.cTypeOne][0],u.castlePos[5+u.cTypeOne][1] ,fheigth;
            u.cTypeOne++;
            break;
          case 9: // x2-y2
            fheigth = 0.119 + (u.legoHeights[blockk])*0.0436;  
            vff <<u.castlePos[0][0],u.castlePos[0][1] ,fheigth;
            break;
          case 5: // y3-z2
            fheigth = 0.115;  
            std::strcpy(u.lastLego[5],"end_table");
            std::strcpy(squareBlockNames[u.cTypeTwo],u.lastLego[5]);
            vff <<u.castlePos[1+u.cTypeTwo][0],u.castlePos[1+u.cTypeTwo][1] ,fheigth;
            u.cTypeTwo++;
            break;
          case 4: // twinfillet
            if(u.cTypeThree==2){
              fheigth = 0.1586 + (u.legoHeights[9])*0.0436;  // sommo un blocco di z2 in più che rappresenta i twinfillet
              vff <<u.castlePos[0][0],u.castlePos[0][1] ,fheigth;
              std::cout << "Sono dentro, vff" <<vff<< std::endl;
            }else{
              fheigth = 0.115 + (u.legoHeights[9])*0.0436; // l'altezza della colonna dei blocchi di tipo 9  
              vff <<u.castlePos[9+u.cTypeThree][0],u.castlePos[9+u.cTypeThree][1] ,fheigth;
              std::strcpy(u.lastLego[4],u.lastLego[9]);
            }
            u.cTypeThree++;
            break;
          case 3: // bandiera  
            std::strcpy(u.lastLego[3],u.lastLego[4]); // copia l'ultimo twinfillet per fare il link dinamico 
            fheigth = 0.2022 + (u.legoHeights[9])*0.0436;  // sommo un blocco di z2 in più che rappresenta i twinfillet
            float deltaBandiera = -0.031;
            vff <<deltaBandiera,u.castlePos[0][1],fheigth;
            break;
        }

        MatrixXf Th;
        take_place_link(dynLinkAtt, dynLinkDet, ur5_joint_array_pub, ee_pos, vff, ee_angle, Th, initial_jnt_pos, blockName, blockk, u, loop_rate, ur5_gripper_pub,gg);
        u.legoHeights[blockk]++;
    }
    return 0;
}
