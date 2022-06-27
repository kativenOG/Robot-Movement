#pragma once
#include "gazebo_ros_link_attacher/Attach.h"
#include "ros/ros.h"
#include "move.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Proviamo a metterli qui :) 
Vector3f STND_POS,STND_ANGLE;

void cleanTh(Eigen::MatrixXf &Th)
{
    for (int i = 0; i < Th.rows(); i++) for (int j = 0; j < Th.cols(); j++) Th(i,j)= 0;
}

void take(ros::ServiceClient attach, ros::Publisher ur5_pub[], Eigen::Vector3f vf, Eigen::Vector3f phiF, Eigen::MatrixXf &Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate)
{

    STND_POS << 0, 0.4403, 0.5147;
    STND_ANGLE << -0.4280, -0.0028, 3.0650;

    gazebo_ros_link_attacher::Attach srv;
    movement(ur5_pub, vf, phiF, Th, initial_pos, u, loop_rate);

    // cout<<"take: "<<(blockName)<<std::endl;    // Per mock test commento il link dinamico

    srv.request.model_name_1 = "ur5";
    srv.request.link_name_1 = "hand_link";
    srv.request.model_name_2 = blockName; //"lego" + to_string(type+1);
    srv.request.link_name_2 = "link";
    attach.call(srv);
    sleep(0.5);

    // Ho accesso a Th
    int rows = Th.rows() - 1;
    VectorXf v(6);
    v << 0, 0, 0, 0, 0, 0; // ma che cazz hahahahah
    // Prendo l'ultima posizione in cui è arrivata con il move di Th, sarebbe più facile se tornasse sempre a una posizione standard ;)
    for (int i = 0; i < 6; i++) v[i] = Th(rows, i + 1);

    cleanTh(Th);
    movement(ur5_pub, STND_POS, STND_ANGLE, Th, v, u, loop_rate);
};

void place(ros::ServiceClient detach, ros::Publisher ur5_pub[], Eigen::Vector3f vf, Eigen::Vector3f phiF, Eigen::MatrixXf &Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate)
{
    STND_POS << 0, 0.4403, 0.5147;
    STND_ANGLE << -0.4280, -0.0028, 3.0650;

    gazebo_ros_link_attacher::Attach srv;
    movement(ur5_pub, vf, phiF, Th, initial_pos, u, loop_rate);

    sleep(0.5);
    srv.request.model_name_1 = "ur5";
    srv.request.model_name_1 = "ur5";
    srv.request.link_name_1 = "hand_link";
    srv.request.model_name_2 = blockName; //"lego" + to_string(type+1);
    srv.request.link_name_2 = "link";
    detach.call(srv);

    // Ritorna posizione standard
    int rows = Th.rows() - 1;
    VectorXf v(6);
    v << 0, 0, 0, 0, 0, 0; // ma che cazz hahahahah
    // Prendo l'ultima posizione in cui è arrivata con il move di Th, sarebbe più facile se tornasse sempre a una posizione standard ;)
    for (int i = 0; i < 6; i++) v[i] = Th(rows, i + 1);

    cleanTh(Th);
    movement(ur5_pub, STND_POS, STND_ANGLE, Th, v, u, loop_rate);
}

void take_and_place(ros::ServiceClient attach, ros::ServiceClient detach, ros::Publisher ur5_pub[], Eigen::Vector3f vf1, Eigen::Vector3f vf2, Eigen::Vector3f phiF, Eigen::MatrixXf Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate)
{

    STND_POS << 0, 0.4403, 0.5147;
    take(attach, ur5_pub, vf1, phiF, Th, initial_pos, blockName, u, loop_rate);

        // int aspetta;
        // std::cout << "terminare??? \n";
        // std::cin >> aspetta;
        // if (aspetta == 1) return;

        // // Non ce ne è più bisogno se torna ogni volta nella posizione standard !
    int rows = Th.rows() - 1;
    VectorXf v(6);
    v << 0, 0, 0, 0, 0, 0; // ma che cazz hahahahah
    // Prendo l'ultima posizione in cui è arrivata con il move di Th, sarebbe più facile se tornasse sempre a una posizione standard ;)
    for (int i = 0; i < 6; i++) v[i] = Th(rows, i + 1);

    place(detach, ur5_pub, vf2, phiF, Th, v, blockName, u, loop_rate);
    cleanTh(Th);
    };
