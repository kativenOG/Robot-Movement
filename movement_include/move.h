#pragma once

void move(ros::Publisher ur5_pub[],Eigen::Vector3f vf,Eigen::Vector3f phiF, Eigen::MatrixXf Th,Eigen::VectorXf initial_pos,robot::ur5 u,ros::Rate loop_rate){
    
    sleep(1);
    ros::spinOnce(); // refresho i valori in initial_pos 

    //orientamento ee finale, è da passare
    //Vector3f v2;
    //v2 << M_PI / 4, M_PI / 4, M_PI / 4;
    // Matrici dell'output del p2pMotionPlan
    // MatrixXf Th;

    VectorXf appo = initial_pos.block(0, 0, 6, 1);

    u.p2pMotionPlan(appo, vf, phiF, Th);

    std_msgs::Float64 temp;
    for (int i = 0; i < Th.rows(); i++)
    {
        for (int j = 1; j < 7; j++)
        {
            temp.data = Th(i, j);
            ur5_pub[j - 1].publish(temp);
            // loop_rate.sleep();
        }
        loop_rate.sleep();
    }
}