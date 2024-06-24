#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;

    // Create a publisher for the "chatter" topic
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);

    // Create a message object
    std_msgs::String msg;
    msg.data = "Hello, World!";

    // Publish the message repeatedly
    ros::Rate rate(1);  // 1 Hz
    while (ros::ok())
    {
        pub.publish(msg);
        ROS_INFO("Published message: %s", msg.data.c_str());
        rate.sleep();
    }

    return 0;
}

