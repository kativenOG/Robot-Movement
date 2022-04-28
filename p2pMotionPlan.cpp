#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include "ur5.h"
#include "inverse.cpp"
//#include "direct.cpp"

using namespace Eigen;
using namespace std;
using namespace robot;


Matrix3f ur5::eul2rotm(Vector3f& v){
    if(v.size()!=3){
        cerr << "wrong vector size" << endl;
        exit(0);
    }

    Matrix3f rotm;
    float s_1 = sin(v(0));
    float c_1 = cos(v(0));
    float s_2 = sin(v(1));
    float c_2 = cos(v(1));
    float s_3 = sin(v(2));
    float c_3 = cos(v(2));

    rotm(0,0) =  c_1*c_2;
    rotm(0,1) =  c_1*s_2*s_3 - s_1*c_3;
    rotm(0,2) =  c_1*s_2*c_3 + s_1*s_3;
    rotm(1,0) =  s_1*c_2;
    rotm(1,1) =  s_1*s_2*s_3 + c_1*c_3;
    rotm(1,2) =  s_1*s_2*c_3 - c_1*s_3;
    rotm(2,0) = -s_2;
    rotm(2,1) =  c_2*s_3;
    rotm(2,2) =  c_2*c_3;

    return rotm;
}

Vector3f ur5::rotm2eul(Matrix3f& m){
    Vector3f v;
    if(m(2,0)<1){
        if(m(2,0)>-1){
            v(0) = atan2(m(1,0),m(0,0));
            v(1) = asin(-m(2,0));
            v(2) = atan2(m(2,1),m(2,2));
        }
        else{
            v(0) = -atan2(-m(1,2),m(1,1));
            v(1) = M_PI/2;
            v(2) = 0;
        }
    }
    else{
        v(0) = atan2(-m(1,2),m(1,1));
        v(1) = -M_PI/2;
        v(2) = 0;
    }
    return v;
}

void ur5::p2pMotionPlan(Vector3f& xEs, Vector3f& phiEs, Vector3f& xEf, Vector3f& phiEf, float minT, float maxT, float deltaT){
    MatrixXf qEs_t = ur5inverse(xEs, eul2rotm(phiEs).inverse());
    MatrixXf qEf_t = ur5inverse(xEf, eul2rotm(phiEf).inverse());
    int s1 =sizeof(qEs_t)/sizeof(qEs_t(0));
    int s2 =sizeof(qEf_t)/sizeof(qEf_t(0));
    RowVectorXf qEs = qEs_t.block(0,0,1,s1);
    RowVectorXf qEf = qEf_t.block(0,0,1,s2);
    MatrixXf A(s1,4);
    for(int i=0;i<s1;i++){
        Matrix4f M;
        M<< 1, minT, pow(minT,2), pow(minT,3),
            0, 1, 2*minT, 3*pow(minT,2),
            1, maxT, pow(maxT,2), pow(maxT,3),
            0, 1, 2*maxT, 3*pow(maxT,2);
        Vector4f b = {qEs(i),0,qEf(i),0};
        Vector4f a = M.inverse()*b;
        A(i,0) = a(0);
        A(i,1) = a(1);
        A(i,2) = a(2);
        A(i,3) = a(3);
    }
    int counter=0;
    int ro = (maxT-minT)/deltaT;
    MatrixXf Th(ro,s1+1);
    MatrixXf xE(ro,4);
    MatrixXf phiE(ro,4);
    for(float i=minT; i<maxT; i+deltaT){
        float j=i;
        VectorXf J(s1+1);
        J(0)=j;
        for(int k=0; i<s1; i++){
            float q = A(k,0)+A(k,1)*i+A(k,2)*i*i+A(k,3)*i*i*i;
            J(k+1)=q;
        }
        for(int k=0; k<s1+1; k++){
            Th(counter,k)=J(k);
        }
        Vector3f x;
        Matrix3f r;
        float t[6] = {J(0),J(1),J(2),J(3),J(4),J(5)};
        ur5direct(t,x,r);
        xE(counter,0)=i;
        xE(counter,1)=x(0);
        xE(counter,2)=x(1);
        xE(counter,3)=x(2);
        Vector3f phi = rotm2eul(r);
        phiE(counter,0)=i;
        phiE(counter,1)=phi(0);
        phiE(counter,2)=phi(1);
        phiE(counter,3)=phi(2);
        counter++;
    }
}



int main(){
    ur5 u;
    Vector3f v; v<<0.3,0.3,0.3;
    Vector3f v1; v1<<0,0,0;
    Vector3f v2; v2<<0.5,0.5,0.5;
    Vector3f v3; v3<<M_PI,M_PI,M_PI;
    u.p2pMotionPlan(v,v1,v2,v3,0,1,0.1);
    return 0;
}