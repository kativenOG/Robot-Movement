#include <iostream>
//#include <Eigen/Dense>
#include <math.h>
#include "ur5.h"
#include "inverse.cpp"

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
    Vector3f x;
    Matrix3f r;
    float y[6];
    for(int i=0; i<6; i++){
        y[i]=qEs(i);
    }
    ur5direct(y, x, r);
    float deltaT=sqrt( pow((x(0)-xEf(0)),2) + pow((x(1)-xEf(1)),2) + pow((x(2)-xEf(2)),2) )/50;
    MatrixXf qEf_t = ur5inverse(xEf, eul2rotm(phiEf).inverse());
    int corners = 6;
    RowVectorXf qEf = qEf_t.block(0, 0, 1, corners);
    MatrixXf A(corners, 4);
    int minT=0;
    int maxT=1;
    for (int i = 0; i < corners; i++){
        Matrix4f M;
        M << 1, minT, pow(minT, 2), pow(minT, 3),
            0, 1, 2 * minT, 3 * pow(minT, 2),
            1, maxT, pow(maxT, 2), pow(maxT, 3),
            0, 1, 2 * maxT, 3 * pow(maxT, 2);
        Vector4f b = {qEs(i), 0, qEf(i), 0};
        Vector4f c = M.inverse() * b;
        A(i, 0) = c(0);
        A(i, 1) = c(1);
        A(i, 2) = c(2);
        A(i, 3) = c(3);
    }
    int counter = 0;
    int ro = (maxT - minT) / deltaT + 1;
    MatrixXf Th(ro, corners + 1);
    float f = 0.00001;
    for (float i = minT; i <= maxT + f; i += deltaT){
        Th(counter,0)=(float)counter / ((maxT - minT) / deltaT);
        for (int k = 0; k < corners; k++){
            Th(counter,k+1)=A(k, 0) + A(k, 1) * i + A(k, 2) * i * i + A(k, 3) * i * i * i;
        }
        counter++;
    }
    Th_1 = Th;
}

/*int main(){
    ur5 u;
    VectorXf v1(6);
    v1 << 3.60739, -0.560554, 2.27689, 2.99605, 1.5708, -2.03659;
    Vector3f v2;
    v2 << 0.5, 0.5, 0.5;
    Vector3f v3;
    v3 << M_PI / 4, M_PI / 4, M_PI / 4;
    MatrixXf Th;
    u.p2pMotionPlan(v1, v2, v3, Th);
    cout << "Th" << endl
         << Th << endl
         << endl;
    return 0;
}*/

//COMMENTARE MAIN E EIGEN LIB PER GITHUB