#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "robot_movement/customMsg.h"
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mockVision");
    ros::NodeHandle n;
    /**
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher pub = n.advertise<robot_movement::customMsg>("brick", 10);
    ros::Rate loop_rate(10);
    int count = 0;
    float x, y, z;
    while (count < 5)
    {
        std::cout << "count= " << count << std::endl;
        std::cout << "insert x: ";
        std::cin >> x;
        std::cout << "insert y: ";
        std::cin >> y;
        std::cout << "insert z: ";
        std::cin >> z;
        robot_movement::customMsg msg;
        msg.x = x;
        msg.y = y;
        msg.z = z;
        msg.r = 0;
        msg.p = 3.14;
        msg.y = 0;
        msg.gWidth = 0.5;
        msg.type = count;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    while (ros::ok())
        return 0;
}
// %EndTag(FULLTEXT)%
