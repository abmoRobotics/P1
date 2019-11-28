#include <iostream>
#include <ros/ros.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <std_msgs/String.h>


void ButtonEventCallback(const kobuki_msgs::ButtonEvent::ConstPtr& msg)
{ 
    ROS_INFO("Button number/state: %d/%d", msg->button, msg->state);
}

int main(int argc, char **argv)
{
    std::cout << "Starter buttons" << std::endl;

    ros::init(argc, argv, "kobuki_button");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/mobile_base/events/button", 1000, ButtonEventCallback);

    ros::spin();

    return 0;
}




