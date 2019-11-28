#include <ros/ros.h>
#include <kobuki_msgs/BumperEvent.h>
#include <iostream>


//Bumper callback
void BumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    if (msg->state == msg->PRESSED)
    {
	ROS_INFO("bruh");
    }
}



// //Main
int main(int argc, char **argv)
{

    system("roslaunch turtlebot_bringup minimal.launch & rviz");

    ros::init(argc, argv, "sans");

    ROS_INFO("Sans");

    //BUMPERS
    ros::NodeHandle bumperNode;

    ros::Subscriber bumper_sub = bumperNode.subscribe("/mobile_base/events/bumper", 1000, BumperCallback);

    ros::spin();

    return 0;
}
