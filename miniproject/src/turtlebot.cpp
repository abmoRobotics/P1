#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/Led.h>

class Turtlebot
{
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    ros::Publisher led1_pub = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1000);
    ros::Publisher led2_pub = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1000);
    ros::Subscriber sub = n.subscribe("/mobile_base/events/button", 1000, &Turtlebot::_button_event,this);
    //ros::ServiceClient client = nh.ServiceClient<> 

    void _button_event(const kobuki_msgs::ButtonEvent::ConstPtr &msg)
    {
        if (msg->state == msg->PRESSED)
        {
            if (msg->button == msg->Button0)
            {
                std::cout << "B0 er nu blevet trykket på" << std::endl;
		move_square();
            }
            if (msg->button == msg->Button1)
            {
                std::cout << "B1 er nu blevet trykket på" << std::endl;
                led_blink();
            }

            if (msg->button == msg->Button2)
            {
                std::cout << "B2 er nu blevet trykket på" << std::endl;
            }
        }
    }

    bool led_on_off = false;


    void led_blink()
    {
        if(led_on_off == false)
            led_on_off = true;
        else if(led_on_off == true)
            led_on_off = false;


        kobuki_msgs::Led msg1;
        kobuki_msgs::Led msg2;
        ros::Rate loop_rate(20);
        while (led_on_off) //when led_on_off == true
        {
            msg1.value = msg1.ORANGE;
            led1_pub.publish(msg1);

            msg2.value = msg2.RED;
            led2_pub.publish(msg2);
            ros::Duration(0.2).sleep();
            
            msg1.value = msg1.BLACK;
            led1_pub.publish(msg1);

            msg2.value = msg2.BLACK;
            led2_pub.publish(msg2);
            ros::Duration(0.2).sleep();
            
            ros::spinOnce();
            loop_rate.sleep(); 
        }
         
        
    }

    void move_square(){
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
		cmd_vel_message.angular.z = 0.64; //Den her skal selv justeres sådan at det er en firkant, ikke stol på matematikken, det er fake news..
		cmd_vel_pub.publish(cmd_vel_message);
		t1 = ros::Time::now().toSec();
	    }
	}


    }

}
;
int main(int argc, char *argv[]){
    std::cout << "Tryk på B0 for at køre i en firkant" << std::endl;

    ros::init(argc, argv, "turtlebot");

    

    Turtlebot t;
    ros::spin();
    return 0;
}
