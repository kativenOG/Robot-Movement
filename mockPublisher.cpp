#include "ros/ros.h"
// #include "std_msgs/String.h"
#include "robot_movement/customMsg.h"
#include <sstream>

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
    int count = 0,x;
    while (ros::ok())
    {
        std::cout<<"inserisci 0:"; 
        std::cin>>x; 
        robot_movement::customMsg msg;
        msg->position[0]= 1 ;
        msg->position[1]= 2 ;
        msg->position[2]= 3 ;
        msg->rpy[0]= 1 ;
        msg->rpy[1]= 3 ;
        msg->rpy[2]= 3 ;
        msg->gWidth = 0.3;
        msg->type = 5;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
// %EndTag(FULLTEXT)%
