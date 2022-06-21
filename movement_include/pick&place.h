#pragma once
#include "gazebo_ros_link_attacher/Attach.h"
#include "move.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

void cleanTh(Eigen::MatrixXf &Th)
{
    for (int i = 0; i < Th.rows(); i++)
    {
        for (int j = 0; j < Th.cols(); j++) Th(i,j) = 0
    }
}

void take(ros::ServiceClient attach, ros::Publisher ur5_pub[], Eigen::Vector3f vf, Eigen::Vector3f phiF, Eigen::MatrixXf &Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate)
{
    // cout<<"take: "<<(blockName)<<std::endl;
    Vector3f STND_POS;
    STND_POS << -0.4064, -0.1403, 0.5147;
    Vector3f STND_ANGLE;
    STND_ANGLE << -0.4280, -0.0028, 3.0650;

    gazebo_ros_link_attacher::Attach srv;
    movement(ur5_pub, vf, phiF, Th, initial_pos, u, loop_rate);

    // Per mock test commento il link dinamico
    // srv.request.model_name_1 = "ur5";
    // srv.request.link_name_1 = "hand_link";
    // srv.request.model_name_2 = blockName; //"lego" + to_string(type+1);
    // srv.request.link_name_2 = "link";
    // attach.call(srv);

    // Ho accesso a Th
    int rows = Th.rows() - 1;
    VectorXf v(6);
    v << 0, 0, 0, 0, 0, 0; // ma che cazz hahahahah
    for (int i = 0; i < 6; i++)
    {
        // prendo l'ultima posizione in cui è arrivata con il move di Th, sarebbe più facile se tornasse sempre a una posizione standard ;)
        v[i] = Th(rows, i + 1);
    }
    // Ritorna posizione standard
    movement(ur5_pub, STND_POS, STND_ANGLE, Th, v, u, loop_rate);
};

void place(ros::ServiceClient detach, ros::Publisher ur5_pub[], Eigen::Vector3f vf, Eigen::Vector3f phiF, Eigen::MatrixXf &Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate)
{
    // cout<<"place: "<<blockName<<std::endl;
    Vector3f STND_POS;
    STND_POS << -0.4064, -0.1403, 0.5147;
    Vector3f STND_ANGLE;
    STND_ANGLE << -0.4280, -0.0028, 3.0650;

    gazebo_ros_link_attacher::Attach srv;

    movement(ur5_pub, vf, phiF, Th, initial_pos, u, loop_rate);

    // Per mock test commento il link dinamico
    // srv.request.model_name_1 = "ur5";
    // srv.request.model_name_1 = "ur5";
    // srv.request.link_name_1 = "hand_link";
    // srv.request.model_name_2 = blockName; //"lego" + to_string(type+1);
    // srv.request.link_name_2 = "link";
    // detach.call(srv);

    // Ritorna posizione standard
    int rows = Th.rows() - 1;
    VectorXf v(6);
    v << 0, 0, 0, 0, 0, 0; // ma che cazz hahahahah
    for (int i = 0; i < 6; i++)
    {
        // prendo l'ultima posizione in cui è arrivata con il move di Th, sarebbe più facile se tornasse sempre a una posizione standard ;)
        v[i] = Th(rows, i + 1);

        movement(ur5_pub, STND_POS, STND_ANGLE, Th, v, u, loop_rate);
    };

    void take_and_place(ros::ServiceClient attach, ros::ServiceClient detach, ros::Publisher ur5_pub[], Eigen::Vector3f vf1, Eigen::Vector3f vf2, Eigen::Vector3f phiF, Eigen::MatrixXf Th, Eigen::VectorXf initial_pos, char *blockName, robot::ur5 u, ros::Rate loop_rate)
    {
        take(attach, ur5_pub, vf1, phiF, Th, initial_pos, blockName, u, loop_rate);

        // cin di wait che dovremmo usare per evitare di perdere dati
        int aspetta;
        std::cout << "terminare??? \n";
        std::cin >> aspetta;
        if (aspetta == 1)
            return;

        // Non ce ne è più bisogno se torna ogni volta nella posizione standard !
        // int rows = Th.rows() - 1;
        // VectorXf v(6);
        // v << 0, 0, 0, 0, 0, 0; // ma che cazz hahahahah
        // for (int i = 0; i < 6; i++)
        // {
        //     // prendo l'ultima posizione in cui è arrivata con il move di Th, sarebbe più facile se tornasse sempre a una posizione standard ;)
        //     v[i] = Th(rows, i + 1);
        // }

        place(detach, ur5_pub, vf2, phiF, Th, STND_POS, blockName, u, loop_rate);
    };
