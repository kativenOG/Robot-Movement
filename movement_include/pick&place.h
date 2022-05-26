#pragma once

Vector3f STND_POS;
STND_POS << -0.4064, -0.1403, 0.5147;
Vector3f STND_ANGLE;
STND_ANGLE << -0.4280, -0.0028, 3.0650;

void take(ros::ServiceClient attach, ros::Publisher ur5_pub[], Eigen::Vector3f vf, Eigen::Vector3f phiF, Eigen::MatrixXf Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate)
{

    gazebo_ros_link_attacher::Attach srv;
    move(ur5_pub, vf, phiF, Th, initial_pos, u, loop_rate);

    sleep(2);
    srv.request.model_name_1 = "ur5";
    srv.request.link_name_1 = "hand_link";
    srv.request.model_name_2 = blockName; //"lego" + to_string(type+1);
    srv.request.link_name_2 = "link";
    attach.call(srv);
    sleep(1);
    move(ur5_pub, STND_POS, STND_ANGLE, Th, initial_pos, u, loop_rate);
};

void place(ros::ServiceClient detach, ros::Publisher ur5_pub[], Eigen::Vector3f vf, Eigen::Vector3f phiF, Eigen::MatrixXf Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate)
{

    gazebo_ros_link_attacher::Attach srv;

    move(ur5_pub, vf, phiF, Th, initial_pos, u, loop_rate);

    sleep(2);
    srv.request.model_name_1 = "ur5";
    srv.request.link_name_1 = "hand_link";
    srv.request.model_name_2 = blockName; //"lego" + to_string(type+1);
    srv.request.link_name_2 = "link";
    detach.call(srv);
    sleep(1);
    move(ur5_pub, STND_POS, STND_ANGLE, Th, initial_pos, u, loop_rate);
};

void take_and_place(ros::ServiceClient attach, ros::ServiceClient detach, ros::Publisher ur5_pub[], Eigen::Vector3f vf1, Eigen::Vector3f vf2, Eigen::Vector3f phiF, Eigen::MatrixXf Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate)
{
    
    take(attach, ur5_pub[], vf1, phiF, Th, initial_pos, blockName, u, loop_rate);
    sleep(1)
    place(detach, ur5_pub[], vf2, phiF, Th, initial_pos, blockName, u, loop_rate);
};