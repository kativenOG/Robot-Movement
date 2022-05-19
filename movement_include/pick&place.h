#pragma once 

void take(ros::ServiceClient attach,ros::Publisher ur5_pub[],Eigen::Vector3f vf,Eigen::Vector3f phiF, Eigen::MatrixXf Th,Eigen::VectorXf initial_pos, char *blockName,robot::ur5 u,ros::Rate loop_rate){

    gazebo_ros_link_attacher::Attach srv;

    move(ur5_pub,vf,phiF,Th,initial_pos,u,loop_rate);

    sleep(2);
    srv.request.model_name_1 = "ur5";
    srv.request.link_name_1 = "hand_link";
    srv.request.model_name_2 = blockName;//"lego" + to_string(type+1);
    srv.request.link_name_2 = "link";
    attach.call(srv);

};
void place(ros::ServiceClient detach,ros::Publisher ur5_pub[],Eigen::Vector3f vf,Eigen::Vector3f phiF, Eigen::MatrixXf Th,Eigen::VectorXf initial_pos, char *blockName,robot::ur5 u,ros::Rate loop_rate){

    gazebo_ros_link_attacher::Attach srv;

    move(ur5_pub,vf,phiF,Th,initial_pos,u,loop_rate);

    sleep(2);
    srv.request.model_name_1 = "ur5";
    srv.request.link_name_1 = "hand_link";
    srv.request.model_name_2 = blockName;//"lego" + to_string(type+1);
    srv.request.link_name_2 = "link";
    detach.call(srv);
};