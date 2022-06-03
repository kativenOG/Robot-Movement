#pragma once
#include "ur5.h" 
using namespace std;
using namespace Eigen;
using namespace robot;

MatrixXf ur5::ur5inverse(Vector3f eep, Matrix3f eer){ // eep: end effector position; eer: end effector rotation matrix
    // T60: matrice di traslazione (il risultato della direct)
    MatrixXf T60f(3,4);
    T60f<<eer,eep;
    Matrix4f T60;
    T60<<T60f,0,0,0,1;
    // lets find the 2 possibile values for th1:
    // p50: distance between frame5 from the frame0 perspective
    MatrixXf p50(4,1);
    Vector4f appo;
    appo <<0,0,-d[5],1;
    p50 = T60 * appo;
    float ac=d[3]/hypot(p50(1),p50(0));
    float th1_1 =atan2(p50(1),p50(0))  +  (M_PI/2);
    float th1_2 =atan2(p50(1),p50(0))  +  (M_PI/2);
    if(ac<=1&&ac>=-1){
        th1_1 =atan2(p50(1),p50(0)) + acos(ac) +  (M_PI/2);
        th1_2 =atan2(p50(1),p50(0)) - acos(ac) +  (M_PI/2);
    }
    // lets find the 4 possibile values for th5:
    ac=(( (eep(0) * sin(th1_1)) - (eep(1) * cos(th1_1)) -d[3])/d[5]);
    float th5_1=0;
    if(ac<=1&&ac>=-1){
        th5_1 = (acos(ac));
    }
    float th5_2 = -th5_1;

    ac=(( (eep(0) * sin(th1_2)) - (eep(1) * cos(th1_2)) -d[3])/d[5]);
    float th5_3=0;
    if(ac<=1&&ac>=-1){
        th5_3 = (acos(ac));
    }
    float th5_4 = -th5_3; 


    Matrix4f T06 = T60.inverse();
    Vector3f Xhat = T06.block(0,0,3,1); // inversa rotazione in x 
    Vector3f Yhat = T06.block(0,1,3,1); // inversarotazione in y 

    // lets find the 4 possibile values for th6:
    float th6_1 =(atan2( ( ( -Xhat(1) * sin(th1_1)) + (Yhat(1) * cos(th1_1) ) )/sin(th5_1) , ( ( Xhat(0) * sin(th1_1)) - (Yhat(0) * cos(th1_1) ) )/sin(th5_1)));
    float th6_2 =(atan2( ( ( -Xhat(1) * sin(th1_1)) + (Yhat(1) * cos(th1_1) ) )/sin(th5_2) , ( ( Xhat(0) * sin(th1_1)) - (Yhat(0) * cos(th1_1) ) )/sin(th5_2)));
    float th6_3 =(atan2( ( ( -Xhat(1) * sin(th1_2)) + (Yhat(1) * cos(th1_2) ) )/sin(th5_3) , ( ( Xhat(0) * sin(th1_2)) - (Yhat(0) * cos(th1_2) ) )/sin(th5_3)));
    float th6_4 =(atan2( ( ( -Xhat(1) * sin(th1_2)) + (Yhat(1) * cos(th1_2) ) )/sin(th5_4) , ( ( Xhat(0) * sin(th1_2)) - (Yhat(0) * cos(th1_2) ) )/sin(th5_4)));


    // matrices for th3: 

    setT10f(th1_1);
    setT65f(th6_1);
    setT54f(th5_1);
    Matrix4f T41m = T10f.inverse() * T60 * T65f.inverse() * T54f.inverse() ;
    Vector3f p41_1 = T41m.block(0,3,3,1); 
    float p41xz_1 = hypot(p41_1(0),p41_1(2));

    setT65f(th6_2);
    setT54f(th5_2);
    T41m = T10f.inverse() * T60 * T65f.inverse() * T54f.inverse() ;
    Vector3f p41_2 = T41m.block(0,3,3,1); 
    float p41xz_2 = hypot(p41_2(0),p41_2(2));

    setT10f(th1_2);
    setT65f(th6_3);
    setT54f(th5_3);
    T41m = T10f.inverse() * T60 * T65f.inverse() * T54f.inverse() ;
    Vector3f p41_3 = T41m.block(0,3,3,1); 
    float p41xz_3 = hypot(p41_3(0),p41_3(2));

    setT65f(th6_4);
    setT54f(th5_4);
    T41m = T10f.inverse() * T60 * T65f.inverse() * T54f.inverse() ;
    Vector3f p41_4 = T41m.block(0,3,3,1); 
    float p41xz_4 = hypot(p41_4(0),p41_4(2));



    // lets find the 8 possibile values for th3:

    ac=(pow(p41xz_1,2) - pow(a[1],2) - pow(a[2],2)) / (2 * a[1] * a[2]);
    float th3_1=0;
    if(ac<=1&&ac>=-1){
        th3_1 =(acos(ac));
    }

    ac=( (pow(p41xz_2,2) - pow(a[1],2) - pow(a[2],2)) / (2 * a[1] * a[2]) );
    float th3_2=0;
    if(ac<=1&&ac>=-1){
        th3_2 =(acos(ac));
    }

    ac=( (pow(p41xz_3,2) - pow(a[1],2) - pow(a[2],2)) / (2 * a[1] * a[2]) );
    float th3_3=0;
    if(ac<=1&&ac>=-1){
        th3_3 =(acos(ac));
    }
    
    ac=( (pow(p41xz_4,2) - pow(a[1],2) - pow(a[2],2)) / (2 * a[1] * a[2]) );
    float th3_4=0;
    if(ac<=1&&ac>=-1){
        th3_4 =(acos(ac));;
    }
    

    float th3_5 = -th3_1; 
    float th3_6 = -th3_2; 
    float th3_7 = -th3_3; 
    float th3_8 = -th3_4; 


    // lets find the 8 possibile values for th2:
    float as =(((-a[2])*sin(th3_1))/p41xz_1);
    float th2_1 =( atan2(-p41_1(2) , -p41_1(0)) ); 
    if(as<=1&&-as>=-1){
        th2_1 =( atan2(-p41_1(2) , -p41_1(0)) - asin(as) );
    }

    as =(((-a[2])*sin(th3_2))/p41xz_1);
    float th2_2 =( atan2(-p41_2(2) , -p41_2(0)) ); 
    if(as<=1&&-as>=-1){
        th2_2 =( atan2(-p41_2(2) , -p41_2(0)) - asin(as) );
    }

    as =(((-a[2])*sin(th3_3))/p41xz_3);
    float th2_3 =( atan2(-p41_3(2) , -p41_3(0)) ); 
    if(as<=1&&-as>=-1){
        th2_3 =( atan2(-p41_3(2) , -p41_3(0)) - asin(as) );
    }

    as =(((-a[2])*sin(th3_4))/p41xz_4);
    float th2_4 =( atan2(-p41_4(2) , -p41_4(0)) ); 
    if(as<=1&&-as>=-1){
        th2_4 =( atan2(-p41_4(2) , -p41_4(0)) - asin(as) );
    }

    as =((a[2]*sin(th3_1))/p41xz_1);
    float th2_5 = ( atan2(-p41_1(2) , -p41_1(0)) );
    if(as<=1&&-as>=-1){
        th2_5 =( atan2(-p41_1(2) , -p41_1(0)) - asin(as) );
    }

    as =((a[2]*sin(th3_2))/p41xz_2);
    float th2_6 =( atan2(-p41_2(2) , -p41_2(0)) );
    if(as<=1&&-as>=-1){
        th2_6 =( atan2(-p41_2(2) , -p41_2(0)) - asin(as) ); 
    }

    as =((a[2]*sin(th3_3))/p41xz_3);
    float th2_7 =( atan2(-p41_3(2) , -p41_3(0)) );
    if(as<=1&&-as>=-1){
        th2_7 =( atan2(-p41_3(2) , -p41_3(0)) - asin(as) );
    }

    as =((a[2]*sin(th3_4))/p41xz_4);
    float th2_8 =( atan2(-p41_4(2) , -p41_4(0)) );
    if(as<=1&&-as>=-1){
        th2_8 =( atan2(-p41_4(2) , -p41_4(0)) - asin(as) );
    }



    // lets find the 8 possibile values for th4:

    setT32f(th3_1);
    setT21f(th2_1);
    setT10f(th1_1);
    setT65f(th6_1);
    setT54f(th5_1);
    Matrix4f T43m = T32f.inverse() * T21f.inverse() * T10f.inverse() * T60 * T65f.inverse() * T54f.inverse();
    Vector3f Xhat43 = T43m.block(0,0,3,1); 
    float th4_1 = (atan2(Xhat43(1),Xhat43(0)));
    setT32f(th3_2);
    setT21f(th2_2);
    setT65f(th6_2);
    setT54f(th5_2);
    T43m = T32f.inverse() * T21f.inverse() * T10f.inverse() * T60 * T65f.inverse() * T54f.inverse() ;
    Xhat43 = T43m.block(0,0,3,1); 
    float th4_2 = (atan2(Xhat43(1),Xhat43(0)));

    setT32f(th3_3);
    setT21f(th2_3);
    setT10f(th1_2);
    setT65f(th6_3);
    setT54f(th5_3);
    T43m = T32f.inverse() * T21f.inverse() * T10f.inverse() * T60 * T65f.inverse() * T54f.inverse() ;
    Xhat43 = T43m.block(0,0,3,1); 
    float th4_3 = (atan2(Xhat43(1),Xhat43(0)));


    setT32f(th3_4);
    setT21f(th2_4);
    setT65f(th6_4);
    setT54f(th5_4);
    T43m = T32f.inverse() * T21f.inverse() * T10f.inverse() * T60 * T65f.inverse() * T54f.inverse() ;
    Xhat43 = T43m.block(0,0,3,1); 
    float th4_4 = (atan2(Xhat43(1),Xhat43(0)));

    setT32f(th3_5);
    setT21f(th2_5);
    setT10f(th1_1);
    setT65f(th6_1);
    setT54f(th5_1);
    T43m = T32f.inverse() * T21f.inverse() * T10f.inverse() * T60 * T65f.inverse() * T54f.inverse() ;
    Xhat43 = T43m.block(0,0,3,1); 
    float th4_5 = (atan2(Xhat43(1),Xhat43(0)));

    setT32f(th3_6);
    setT21f(th2_6);
    setT65f(th6_2);
    setT54f(th5_2);
    T43m = T32f.inverse() * T21f.inverse() * T10f.inverse() * T60 * T65f.inverse() * T54f.inverse() ;
    Xhat43 = T43m.block(0,0,3,1); 
    float th4_6 = (atan2(Xhat43(1),Xhat43(0)));

    setT32f(th3_7);
    setT21f(th2_7);
    setT10f(th1_2);
    setT65f(th6_3);
    setT54f(th5_3);
    T43m = T32f.inverse() * T21f.inverse() * T10f.inverse() * T60 * T65f.inverse() * T54f.inverse() ;
    Xhat43 = T43m.block(0,0,3,1); 
    float th4_7 = (atan2(Xhat43(1),Xhat43(0)));

    setT32f(th3_8);
    setT21f(th2_8);
    setT65f(th6_4);
    setT54f(th5_4);
    T43m = T32f.inverse() * T21f.inverse() * T10f.inverse() * T60 * T65f.inverse() * T54f.inverse() ;
    Xhat43 = T43m.block(0,0,3,1); 
    float th4_8 = (atan2(Xhat43(1),Xhat43(0)));



    // Matrice delle 8 combinazioni di angoli:

    MatrixXf Th(8,6);

    Th << th1_1 , th2_1, th3_1 , th4_1, th5_1 , th6_1,  
          th1_1 , th2_2, th3_2 , th4_2, th5_2 , th6_2,  
          th1_2 , th2_3, th3_3 , th4_3, th5_3 , th6_3,  
          th1_2 , th2_4, th3_4 , th4_4, th5_4 , th6_4,  
          th1_1 , th2_5, th3_5 , th4_5, th5_1 , th6_1,  
          th1_1 , th2_6, th3_6 , th4_6, th5_2 , th6_2,  
          th1_2 , th2_7, th3_7 , th4_7, th5_3 , th6_3,  
          th1_2 , th2_8, th3_8 , th4_8, th5_4 , th6_4;

    return Th;
}
