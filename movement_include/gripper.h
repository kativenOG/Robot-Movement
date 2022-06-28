#pragma once
#define MAX_LENGHT -1

void  closeGripper(Ros::publisher gripper,float size, Ros:Rate loop_rate){
    size = (size/MAX_LENGHT) - 0.5;
    gripper.publish(size);
};

void  openGripper(Ros::publisher gripper, Ros:Rate loop_rate){
    gripper.publish(-0.5);
};
