#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/Led.h>

int led1Value, led2Value = 0;
int chosenColor = 1;

//Callbacks
void Led1Callback(const kobuki_msgs::Led::ConstPtr &msg)
{
    led1Value = msg->value;
    //ROS_INFO("Led1 is: %d", led1Value);
}

void Led2Callback(const kobuki_msgs::Led::ConstPtr &msg)
{
    led2Value = msg->value;
    //ROS_INFO("Led2 is: %d", led2Value);
}

void ButtonCallback(const kobuki_msgs::ButtonEvent::ConstPtr& msg)
{ 
    chosenColor = msg->button + 1;
    ROS_INFO("Color: %d", chosenColor);
}

//Main
int main(int argc, char **argv)
{

    std::cout << "Starter LED'er" << std::endl;

    ros::init(argc, argv, "led_talker");

    ros::NodeHandle buttonNode;
    
    ros::Subscriber button_sub = buttonNode.subscribe("/mobile_base/events/button", 1000, ButtonCallback);

    //Subscribe to Leds
    ros::NodeHandle subNode1;
    ros::NodeHandle subNode2;

    ros::Subscriber sub1 = subNode1.subscribe("/mobile_base/commands/led1", 1000, Led1Callback);
    ros::Subscriber sub2 = subNode2.subscribe("/mobile_base/commands/led2", 1000, Led2Callback);

    //Publish to Leds
    ros::NodeHandle pubNode1;
    ros::NodeHandle pubNode2;

    ros::Publisher led1_pub = pubNode1.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1000);
    ros::Publisher led2_pub = pubNode2.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1000);

    ros::Rate loop_rate(20);

    int count = 0;
    while (ros::ok())
    {

        kobuki_msgs::Led msg1;
        kobuki_msgs::Led msg2;

        if (led1Value == 0){
            msg1.value = chosenColor;
            msg2.value = msg2.BLACK;
        }
        else {
            msg1.value = msg1.BLACK;
            msg2.value = chosenColor;
        }

        //ROS_INFO("Led 1: %d", msg1.value);
        //ROS_INFO("Led 2: %d", msg2.value);

        led1_pub.publish(msg1);
        led2_pub.publish(msg2);

        ros::spinOnce(); //Useful if another node subscriber to get callbacks

        loop_rate.sleep(); //Sleep until loop_rate time passes

        ++count;
    }

    return 0;
}