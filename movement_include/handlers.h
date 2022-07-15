#pragma once 

#define RATE 10  // 10Hz
#define QUEUE_SIZE 11 // salvo 100 blocchi in Buffer

//                      ### JOINTS ###
// Per prendere i valori dei joint dai topic di ros_control
VectorXf initial_jnt_pos(7);
void shoulder_pan_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[0] = val->set_point;
}
void shoulder_lift_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[1] = val->set_point;
}
void elbow_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[2] = val->set_point;
}
void wrist_1_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[3] = val->set_point;
}
void wrist_2_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[4] = val->set_point;
}
void wrist_3_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[5] = val->set_point;
}
void gripper_getter(const control_msgs::JointControllerState::ConstPtr &val)
{
    initial_jnt_pos[6] = val->set_point;
}

//                  ### VISIONE ###
// Salva in matrici i valori inviati tramite messaggi dal topic di visione 
int cnt=0;
MatrixXf block_position(100,3);
VectorXd blockNumber(100);
MatrixXf block_angle(100,3);
VectorXf gripperWidth(100);
void brick_getter(const robot_movement::customMsg::ConstPtr &val)
{
    // Position
    block_position(cnt,0) = (val->x);
    block_position(cnt,1) = (val->y);
    block_position(cnt,2) = (val->z);

    // Orientation
    block_angle(cnt,0) = (val->y_1);
    block_angle(cnt,1) = (val->p);
    block_angle(cnt,2) = (val->r);

    // Block Type and Gripper Width
    blockNumber[cnt] = (val->type);
    gripperWidth[cnt] = (val->gWidth);
    std::cout<<cnt<<" esimo blocco:"<<"  Tipo: "<<blockNumber(cnt)<<"Position: "<< block_position.row(cnt)<<"Angle: "<<block_angle.row(cnt)<<std::endl;
    cnt++;
}
