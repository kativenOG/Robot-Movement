#pragma once
#define MAX_LENGHT -1

void  closeGripper(Ros::publisher gripper,float size, Ros:Rate loop_rate){
    
    size = (size/MAX_LENGHT) - 0.5;
    opengripper.publish(size);
    ros::spinOnce();
    loop_rate.sleep();
};

void  openGripper(Ros::publisher gripper, Ros:Rate loop_rate){
    
    opengripper.publish(-0.5);
    ros::spinOnce();
    loop_rate.sleep();
};