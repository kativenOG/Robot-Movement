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

// Link attacher and movement
#include "gazebo_ros_link_attacher/Attach.h"
#include "move.h"
#include "pick&place.h"

#define RATE 10       // 10Hz
#define QUEUE_SIZE 100 // salvo 100 blocchi in Buffer
using namespace Eigen;
using namespace std;
using namespace robot;

ur5 u;

//                  ### Ros Control ###
VectorXf initial_jnt_pos(7);
void shoulder_pan_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[0] = val->set_point;
}
void shoulder_lift_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[1] = val->set_point;
}
void elbow_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[2] = val->set_point;
}
void wrist_1_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[3] = val->set_point;
}
void wrist_2_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[4] = val->set_point;
}
void wrist_3_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[5] = val->set_point;
}
void gripper_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[6] = val->set_point;
}

//                  ### VISIONE ###
VectorXf block_position(3);
int blockNumber;
VectorXf block_angle(3);
float gripperWidth;
void brick_getter(const robot_movement::customMsg::ConstPtr &val)
{

    // Position
    block_position[0] = (val->x);
    block_position[1] = (val->y);
    block_position[2] = (val->z);

    // Orientation
    block_angle[0] = (val->r);
    block_angle[1] = (val->p);
    block_angle[2] = (val->y);

    // Block Type and Gripper Width
    blockNumber = (val->type);
    gripperWidth = (val->gWidth);
}

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
    ur5_gripper_pub = n.advertise<std_msgs::Float64>("/gripper_controller/command", QUEUE_SIZE);

    // creo subscriber che ascoltano nei topic di poszione dei joint, e si salvano la loro poszione nello spazio tramite dei wrapper :)
    ros::Subscriber shoulder_pan_joint_sub = n.subscribe("/shoulder_pan_joint_position_controller/state", QUEUE_SIZE, shoulder_pan_getter);
    ros::Subscriber shoulder_lift_joint_sub = n.subscribe("/shoulder_lift_joint_position_controller/state", QUEUE_SIZE, shoulder_lift_getter);
    ros::Subscriber elbow_joint_sub = n.subscribe("/elbow_joint_position_controller/state", QUEUE_SIZE, elbow_getter);
    ros::Subscriber wrist_1_joint_sub = n.subscribe("/wrist_1_joint_position_controller/state", QUEUE_SIZE, wrist_1_getter);
    ros::Subscriber wrist_2_joint_sub = n.subscribe("/wrist_2_joint_position_controller/state", QUEUE_SIZE, wrist_2_getter);
    ros::Subscriber wrist_3_joint_sub = n.subscribe("/wrist_3_joint_position_controller/state", QUEUE_SIZE, wrist_3_getter);
    ros::Subscriber left_knucle_joint_sub = n.subscribe("/gripper_joint_position/state", QUEUE_SIZE, gripper_getter); // se è aperto o chiuso (non proprio un angolo )
    // in teoria RATE è la queue size, controlla meglio
    // Vision topic with brick Data
    ros::Subscriber kinect_name = n.subscribe("brick", QUEUE_SIZE, brick_getter);
    int x;

    // METTI SEMPRE COUT PER SINCRONIZZARE COSÌ NON SI PERDONO MESSAGGI
    cout << "Inizio Programma !";
    cin >> x;

    sleep(1); // Startup Perdo informazioni dai topic? Non lo so fratm :(
    // Nodi per il dynamic linker
    ros::ServiceClient dynLinkAtt = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    ros::ServiceClient dynLinkDet = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    for (int i = 0; i < 11; i++)
    {
        cout << "Prossimo blocco!";
        cin >> x;

        // Indispensabili per prendere il blocco successivo
        ros::spinOnce();
        loop_rate.sleep();

        // Cerco il tipo di blocco per capire la posizione finale !!!
        Vector3f vff;
        vff << u.legoPos[blockNumber][0], u.legoPos[blockNumber][1], 0.15;

        MatrixXf Th;
        take_and_place(dynLinkAtt, dynLinkDet, ur5_joint_array_pub, block_position, vff, block_angle, Th, initial_jnt_pos, u.legos[blockNumber], u, loop_rate);
        //                                  *.CONTROLLI.*
        // cout<<block_position(i,0)<<"  "<<block_position(i,1)<<" "<<block_position(i,2)<<endl;
        // cout<<u.legos[(int)(blockNumber)]<<endl;
    }
    return 0;
}
