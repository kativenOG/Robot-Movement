#pragma once 
#include "ur5.h" 

using namespace std;
using namespace Eigen;

void robot::ur5::setT10f(float th1){
    T10f<<   cos(th1),-sin(th1),0,0, 
             sin(th1),cos(th1),0,0,
             0,0,1,d[0],
             0,0,0,1;
};
void robot::ur5::setT21f(float th2){
    T21f<<   cos(th2),-sin(th2),0,0, 
             0,0,-1,0,
             sin(th2),cos(th2),0,0,
             0,0,0,1;
};
void robot::ur5::setT32f(float th3){
    T32f<<   cos(th3),-sin(th3),0,a[1], 
             sin(th3),cos(th3),0,0,
             0,0,1,d[2],
             0,0,0,1;
};
void robot::ur5::setT43f(float th4){
    T43f<<   cos(th4),-sin(th4),0,a[2], 
             sin(th4),cos(th4),0,0,
             0,0,1,d[3],
             0,0,0,1;
};
void robot::ur5::setT54f(float th5){
    T54f<<   cos(th5),-sin(th5),0,0, 
             0,0,-1,-d[4],
             sin(th5),cos(th5),0,0,
             0,0,0,1;
};
void robot::ur5::setT65f(float th6){
    T65f<<   cos(th6),-sin(th6),0,0, 
             0,0,1,d[5],
             -sin(th6),-cos(th6),0,0,
             0,0,0,1;
};

void robot::ur5::ur5direct(VectorXf& th,Vector3f &x,Matrix3f &r){ // i parametri sono gli angoli dei joint e le matrici che deve ritornare 
    setT10f(th(0));
    setT21f(th(1));
    setT32f(th(2));
    setT43f(th(3));
    setT54f(th(4));
    setT65f(th(5));


    // calcolo matrice finale EE
    MatrixXf T06 = T10f * T21f * T32f * T43f * T54f * T65f;
    //MatrixXd T06 = T10f*T21f*T32f*T43f*T54f*T65f;

    x = T06.block(0,3,3,1); // Posizione del end effector 
    r = T06.block(0,0,3,3); // Matrice di Eulero 

    // Debugging 
    //printf("Pozione xe del EE: %f \n",x);
    //printf("Angolazione del EE: %f \n",r);
}

