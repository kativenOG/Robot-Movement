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
#include <std_srvs/Empty.h>
// Movement Kinematics
#include "p2pMotionPlan.h"
#include "ur5.h"
#include "direct.h"
#include "inverse.h"
#include "handlers.h"

// Link attacher and movement
#include "gazebo_ros_link_attacher/Attach.h"
#include "move.h"
// #include "pick&place.h"
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
    ros::Subscriber left_knucle_joint_sub = n.subscribe("/gripper_joint_position/state", QUEUE_SIZE, gripper_getter); // se ?? aperto o chiuso (non proprio un angolo )

    // Vision topic with brick Data
    ros::Subscriber kinect_name = n.subscribe("brick", 11, brick_getter);

    // ASYNC SPINNING 
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty emptySrv;
    pauseGazebo.call(emptySrv);
    // METTI SEMPRE COUT PER SINCRONIZZARE COS?? NON SI PERDONO MESSAGGI
    // int x;
    // cout << "Inizio Programma !";
    // cin >> x;

    // Nodi per il dynamic linker
    ros::ServiceClient dynLinkAtt = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    ros::ServiceClient dynLinkDet = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    for (int i = 0; i < 11; i++)
    {
        while(cnt==i){
          std::cout << "cnt: " <<cnt<<"  i: "<<i<< std::endl;
          sleep(1);
        }
        // Cerco il tipo di blocco per capire la posizione finale !!!
        int blockk = blockNumber[i];
        float gg= gripperWidth[i];
        int rtype= (int)(block_angle(i,2)/10);
        float fheigth = 0.115;
        if(blockk==7 || blockk==1) fheigth= 0.105; 
        Vector3f vff;
        vff << -u.legoPos[blockk][0], -u.legoPos[blockk][1], fheigth;

        // Carico i valori della iesima riga dentro un appoggio da caricare nella funzione pick&place
        Vector3f ee_pos;
        ee_pos<< block_position(i,0),block_position(i,1),block_position(i,2);
        Vector3f ee_angle;

        int ee_roll_appo = ((int)(block_angle(i,2)*1000))%10000;
        float ee_roll = ee_roll_appo/1000;

        if(ee_roll<1) ee_roll=0;
        else if(ee_roll>2) ee_roll=3.14;
        else ee_roll=1.57;

        ee_angle<< block_angle(i,0),block_angle(i,1),ee_roll;

        MatrixXf Th;
        take_place_link(dynLinkAtt, dynLinkDet, ur5_joint_array_pub, ee_pos, vff, ee_angle, Th, initial_jnt_pos, u.legos[blockk],blockk, u, loop_rate, ur5_gripper_pub,gg,rtype,0);
    }
    return 0;
}
