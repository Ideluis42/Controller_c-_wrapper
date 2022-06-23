#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <ostream>


int counter = 0;
void talker(float pos, int argc, char **argv)
{
    ros::init(argc, argv, "comp_talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("comp_talker", 1000);

    ros::Rate loop_rate(20);


    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Float32 msg;
    msg.data = pos;

    ROS_INFO("%f", msg.data);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++counter;
}