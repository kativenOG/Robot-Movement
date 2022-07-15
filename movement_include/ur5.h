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

        // lego height : quanti blocchi per ogni posizione finale 
        float legoHeights [11] = {0,0,0,0,0,0,0,0,0,0,0};

        char lastLego [11][30]={
          "end_table",
          "end_table",
          "end_table",
          "end_table",
          "end_table",
          "end_table",
          "end_table",
          "end_table",
          "end_table",
          "end_table",
          "end_table",
        }; 
        // lego pos
        float legoPos [11][2] = {{-0.52,0.37},{-0.68,-0.23},{-0.675,-0.389},{-0.67,0.075},{-0.675,-0.077},{-0.549,-0.24},{-0.665,0.227},{-0.548,0.064},{-0.545,-0.1072},{-0.543,0.218},{-0.56,-0.39}};

        //lego angles   
        float legoAngle [11][3] = {{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0,M_PI,0},{0.82,M_PI,0},{0.851,M_PI,0},{0,M_PI,0},{0,M_PI,0}};         

        // Valori castello 
        float castePos [11][2]=  {{-0.002,0.531},{-0.11,0.531},{-0.002,0.651},{0.11,0.531},{-0.002,0.419},{-0.071,0.607},{0.071,0.607},{0.071,0.460},{-0.071,0.460},{-0.019,0.531},{0.019,0.531}}; 
        //                           CENTER          Y3_1            Y3_2          Y3_3         Y3_4           Y4_1            Y4_2          Y4_3          Y4_4      FILLET_LEFT    FILLET_RIGHT
        float casteAngle [2][3]={{0,M_PI,0},{M_PI_2,M_PI,0},{2.38,M_PI,0},{-2.38,M_PI,0}}; 
        //                                                      DESTRA        SINISTRA 
        // Magari da portare fuori (globali) in pickPlaceLink per permettere di controllare se sono superiori al primo :)    
        int cTypeOne=0,cTypeTwo=0,cTypeThree=0;

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
