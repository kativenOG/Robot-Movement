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
             0,
             -0.425,
             -0.3922,
             0,
             0,
             0
        };

        const float d[6] = {
                 0.1625,
                 0,
                 0,
                 0.1333,
                 0.0997,
                //  0.2996
                0.1796 //Era 0,1997
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

        // lego pos
        // float legoPos [11][2] = {{0.7,0.3},{0.65,0.15},{0.67,-0.3},{0.65,0},{0.65,-0.15},{0.52,0.3},{0.7,-0.45},{0.52,0.15},{0.52,0},{0.52,-0.15},{0.54,-0.3}};
        // float legoPos [11][2] = {{-0.54,0.45},{-0.7,-0.3},{-0.65,-0.15},{-0.65,0},{-0.65,0.15},{-0.67,0.3},{-0.52,-0.3},{-0.52,-0.15},{-0.52,0},{-0.52,0.15},{-0.52,0.3}};
        float legoPos [11][2] = {{-0.54,0.45},{-0.65,-0.15},{-0.7,-0.3},{-0.65,0.15},{-0.65,0},{-0.52,-0.15},{-0.67,0.3},{-0.52,0.15},{-0.52,0},{-0.54,0.3},{-0.52,-0.3}};

        //lego angles   
        float legoAngle [11][3] = {{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0}};         

        // lego names 
        char legos [11][30]  = {
          "X1-Y1-Z2",
          "X1-Y2-Z1",
          "X1-Y2-Z2",
          "X1-Y2-Z2-CHAMFER",
          "X1-Y2-Z2-TWINFILLET",
          "X1-Y3-Z2",
          "X1-Y3-Z2-FILLET",
          "X1-Y4-Z1",
          "X1-Y4-Z2",
          "X2-Y2-Z2",
          "X2-Y2-Z2-FILLET"}; 

        // Direct kinematics function:
        void ur5direct(VectorXf &th, Vector3f &x, Matrix3f &r);

        // Direct kinematics function:  
        MatrixXf ur5inverse(Vector3f eep, Matrix3f eer);

        void p2pMotionPlan(VectorXf &qEs, Vector3f &xEf, Vector3f &phiE0, MatrixXf &Th_1);
    };
};
