#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/SensorState.h>

class Turtlebot
{
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

    void _button_event(const kobuki_msgs::ButtonEvent::ConstPtr &msg)
    {
        if (msg->state == msg->RELEASED)
        {
            if (msg->button == msg->Button0)
            {
                std::cout << "B0 er nu blevet trykket på" << std::endl;
                for (int i = 0; i < 4; i++)
                {
                    geometry_msgs::Twist cmd_vel_message;
                    cmd_vel_message.linear.x = 1;
                    cmd_vel_message.angular.z = 1.57079633;
                    cmd_vel_pub.publish(cmd_vel_message);
                    ros::Duration(0.5).sleep();
                }
            }
            if (msg->button == msg->Button1)
            {
                std::cout << "B1 er nu blevet trykket på" << std::endl;
            }

            if (msg->button == msg->Button2)
            {
                std::cout << "B2 er nu blevet trykket på" << std::endl;
            }
        }
    }

    Turtlebot()
    {
    }

}

int main(int argc, char *argv[]){
    std::cout << "Tryk på B0 for at køre i en firkant" << std::endl;

    ros::init(argc, argv, "Square");

    return 0;
}
