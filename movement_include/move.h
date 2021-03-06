#pragma once

void movement(ros::Publisher ur5_pub[], Eigen::Vector3f vf, Eigen::Vector3f phiF, Eigen::MatrixXf& Th, Eigen::VectorXf initial_pos, robot::ur5 u, ros::Rate loop_rate, int w)
{
    sleep(1);
    ros::spinOnce(); // refresho i valori in initial_pos
    VectorXf appo = initial_pos.block(0, 0, 6, 1);
    // std::cout<<"pre p2p \n";
    u.p2pMotionPlan(appo, vf, phiF, Th, w);
    // std::cout<<"post p2p \n";
    std_msgs::Float64 temp;
    for (int i = 0; i < Th.rows(); i++)
    {
        for (int j = 1; j < 7; j++)
        {
            // std::cout<<j;
            temp.data = Th(i, j);
            ur5_pub[j - 1].publish(temp);
            // loop_rate.sleep();
        }
        loop_rate.sleep();
    }
}