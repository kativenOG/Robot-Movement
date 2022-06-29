#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "robot_movement/customMsg.h"
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mock_node");
    ros::NodeHandle n;
    
    ros::Publisher pub = n.advertise<robot_movement::customMsg>("brick", 10);
    ros::Rate loop_rate(10);
    int count = 0,type;
    float x, y, z,gripW;
    bool exit;
    while (ros::ok())
    {
        std::cout << "count= " << count << std::endl;
        std::cout << "insert x: ";
        std::cin >> x;
        std::cout << "insert y: ";
        std::cin >> y;
        std::cout << "insert z: ";
        std::cin >> z;
        std::cout << "insert block type: " << std::endl;
        std::cin >> type;
        std::cout << "insert gripper width: " << std::endl;
        std::cin >> gripW;
        robot_movement::customMsg msg;
        msg.x = x;
        msg.y = y;
        msg.z = z;
        msg.r = 0.8;
        msg.p = 3.14;
        msg.y_1 = 0;
        msg.gWidth = gripW;
        msg.type = type;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
  }

  return 0;
}
