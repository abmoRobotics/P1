#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/SensorState.h>

class Turtlebot
{
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    ros::Subscriber sub = n.subscribe("/mobile_base/events/button", 1000, &Turtlebot::_button_event, this);

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

                    double t0 = ros::Time::now().toSec();
                    double t1 = 0;
                    while ((t1 - t0) < 3)
                    {

                        cmd_vel_message.linear.x = 0.3;
                        cmd_vel_message.angular.z = 0;
                        cmd_vel_pub.publish(cmd_vel_message);
                        t1 = ros::Time::now().toSec();
                    }
                    
                    t0 = ros::Time::now().toSec();
                    t1 = 0;
                    while ((t1 - t0) < 3)
                    {

                        cmd_vel_message.linear.x = 0;
                        cmd_vel_message.angular.z = 0.523598776;
                        cmd_vel_pub.publish(cmd_vel_message);
                        t1 = ros::Time::now().toSec();
                    }
                }

                /*
                for (int y=0; y<4; y++){
                    cmd_vel_message.linear.x = 0;
                    cmd_vel_message.angular.z = 1.57079632679;
                    cmd_vel_pub.publish(cmd_vel_message);
                    ros::Duration(0.5).sleep();
                    ros:Rate loop_rate(10);
                }
                */
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
};

int main(int argc, char *argv[])
{
    std::cout << "Tryk på B0 for at køre i en firkant" << std::endl;

    ros::init(argc, argv, "Square");

    Turtlebot jalla;
    ros::spin();
    return 0;
}
