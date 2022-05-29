#pragma once

using namespace std;
using namespace Eigen;

namespace robot
{
    class ur5
    {
    private:
        // DH parameters
        const float a[6] = {
        //    0,
        //    -0.42500,
        //    -0.39225,
        //    0,
        //    0,
        //    0
             
             0,
             -0.425,
             -0.3922,
             0,
             0,
             0
        };

        const float d[6] = {
                 /*0.0892,*/0.1625,
                 0,
                 0,
                 /*0.1093,*/0.1333,
                 /*0.09475,*/0.0997,
                 /*0.0825*/0.0996
            //0.0891589,
            //0,
            //0,
            //0.10915,
            //0.9456,
            //0.0823};
        };

        // Matrici per direct kinematics
        Matrix4f T10f;
        Matrix4f T21f;
        Matrix4f T32f;
        Matrix4f T43f;
        Matrix4f T54f;
        Matrix4f T65f;

        // Setters Handlers:
        void setT10f(float th1);
        void setT21f(float th2);
        void setT32f(float th3);
        void setT43f(float th4);
        void setT54f(float th5);
        void setT65f(float th6);

        Matrix3f eul2rotm(Vector3f &v);
        Vector3f rotm2eul(Matrix3f &m);

    public:
        // Direct kinematics function:
        void ur5direct(VectorXf &th, Vector3f &x, Matrix3f &r);

        // Direct kinematics function:
        MatrixXf ur5inverse(Vector3f eep, Matrix3f eer);

        void p2pMotionPlan(VectorXf &qEs, Vector3f &xEf, Vector3f &phiE0, MatrixXf &Th_1);
    };
};