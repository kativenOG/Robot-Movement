#pragma once
#include "ur5.h"

using namespace Eigen;
using namespace std;
using namespace robot;

Matrix3f ur5::eul2rotm(Vector3f &v)
{
    Matrix3f rotm;
    float s_1 = sin(v(0));
    float c_1 = cos(v(0));
    float s_2 = sin(v(1));
    float c_2 = cos(v(1));
    float s_3 = sin(v(2));
    float c_3 = cos(v(2));

    rotm(0, 0) = c_1 * c_2;
    rotm(0, 1) = c_1 * s_2 * s_3 - s_1 * c_3;
    rotm(0, 2) = c_1 * s_2 * c_3 + s_1 * s_3;
    rotm(1, 0) = s_1 * c_2;
    rotm(1, 1) = s_1 * s_2 * s_3 + c_1 * c_3;
    rotm(1, 2) = s_1 * s_2 * c_3 - c_1 * s_3;
    rotm(2, 0) = -s_2;
    rotm(2, 1) = c_2 * s_3;
    rotm(2, 2) = c_2 * c_3;

    return rotm;
}

Vector3f ur5::rotm2eul(Matrix3f &m){
    Vector3f v;
    if (m(2, 0) < 1){
        if (m(2, 0) > -1){
            v(0) = atan2(m(1, 0), m(0, 0));
            v(1) = asin(-m(2, 0));
            v(2) = atan2(m(2, 1), m(2, 2));
        }
        else{
            v(0) = -atan2(-m(1, 2), m(1, 1));
            v(1) = M_PI / 2;
            v(2) = 0;
        }
    }
    else{
        v(0) = atan2(-m(1, 2), m(1, 1));
        v(1) = -M_PI / 2;
        v(2) = 0;
    }
    return v;
}


void ur5::p2pMotionPlan(VectorXf &qEs, Vector3f &xEf, Vector3f &phiEf, MatrixXf &Th_1){

    //Version 2.0
    MatrixXf qEf_t = ur5inverse(xEf, eul2rotm(phiEf).inverse());
    int corners = 6;
    RowVectorXf qEf = qEf_t.block(0, 0, 1, corners);
    int minT=0;
    int maxT=1;
    //distance control
    for(int i=0; i<corners; i++){
        while(qEs(i)>M_PI){
            qEs(i)-=(2*M_PI);
        }
        while(qEs(i)<-M_PI){
            qEs(i)+=(2*M_PI);
        }
        while(qEf(i)>M_PI){
            qEf(i)-=(2*M_PI);
        }
        while(qEf(i)<-M_PI){
            qEf(i)+=(2*M_PI);
        }
    }
    float deltaT=0;
    for(int i=0; i<corners; i++){
        float a=abs(qEf(i)-qEs(i));
        if(a>deltaT){
            deltaT=a;
        }
    }
    //velocity control
    float vel=abs(qEf(1)-qEs(1)+qEf(2)-qEs(2));
    if(vel>deltaT){
        deltaT=vel;
    }
    deltaT = 4* M_PI / (360 * deltaT);
    int ro = (maxT - minT) / deltaT + 1;
    float f2 = 0.00001;
    MatrixXf Th(ro, corners + 1);
    int counter = 0;
    for (float i = minT; i <= maxT + f2; i += deltaT){
        Th(counter,0)=(float)counter / ((maxT - minT) / deltaT);
        for (int k = 0; k < corners; k++){
            Th(counter,k+1)=qEs(k)+counter*(qEf(k)-qEs(k))/(ro-1);
        }
        counter++;
    }
    Th_1 = Th;
}
