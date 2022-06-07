#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Movement Kinematics 
#include "p2pMotionPlan.h"
#include "ur5.h"
#include "direct.h"
#include "inverse.h"
#include "pick&place.h"
// Link attacher and movement
#include "gazebo_ros_link_attacher/Attach.h"
#include "move.h"

//#include "pick&place.h"

#define RATE 10

using namespace Eigen;
using namespace std;
using namespace robot;


// vector<control_msgs::JointControllerState::ConstPtr> initial_jnt_pos;
VectorXf initial_jnt_pos(7);
ur5 u;

void shoulder_pan_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[0] = val->set_point;
    // initial_jnt_pos[0]=val->process_value;
}
void shoulder_lift_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[1] = val->set_point;
    // initial_jnt_pos[1]=val->process_value;
}
void elbow_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[2] = val->set_point;
    // initial_jnt_pos[2]=val->process_value;
}
void wrist_1_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[3] = val->set_point;
    // initial_jnt_pos[3]=val->process_value;
}
void wrist_2_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[4] = val->set_point;
    // initial_jnt_pos[4]=val->process_value;
}
void wrist_3_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[5] = val->set_point;
    // initial_jnt_pos[5]=val->process_value;
}
void gripper_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[6] = val->set_point;
    // initial_jnt_pos[6]=val->process_value;
}

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
    sleep(1);

    // Indispensabili 
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    loop_rate.sleep();


    // i client per comunicare il link dinamico con il modulo di gazebo 
    ros::ServiceClient dynLinkAtt = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    ros::ServiceClient dynLinkDet = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    float x,y,z;
    float ex,ey,ez;
    cout<<"inserisci orientamento dell'ee:"<<endl;
    cin>>ex;
    cin>>ey;
    cin>>ez;
    cout<<"inserisci valori posizione:"<<endl;
    cin>>x;
    cin>>y;
    cin>>z;

    Vector3f vf;
    vf<<x,y,z;
    Vector3f phiF;
    phiF << ex, ey, ez;
    MatrixXf Th;
    movement(ur5_joint_array_pub, vf, phiF, Th, initial_jnt_pos, u, loop_rate);
    
    // take(dynLinkAtt,ur5_joint_array_pub,vf, phiF, Th, initial_jnt_pos,  "X2-Y2-Z2" ,  u, loop_rate);

    return 0;
}
