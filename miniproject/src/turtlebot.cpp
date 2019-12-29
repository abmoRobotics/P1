#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/Led.h>
#include <std_srvs/Trigger.h>
#include <sound_play/sound_play.h>

class Turtlebot
{
    //NodeHandle
    ros::NodeHandle n;
    //Publishers
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    ros::Publisher led1_pub = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1000);
    ros::Publisher led2_pub = n.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1000);
    //Subscribers
    ros::Subscriber sub = n.subscribe("/mobile_base/events/button", 1000, &Turtlebot::_button_event,this);
    //SoundClient
    sound_play::SoundClient sc; 

    //Button Event Function
    void _button_event(const kobuki_msgs::ButtonEvent::ConstPtr &msg)
    {
        //When a button is pressed
        if (msg->state == msg->PRESSED)
        {
	    //Check which button
            if (msg->button == msg->Button0)
            {
                std::cout << "B0 has been pressed" << std::endl;
		//move_square();
            }
            if (msg->button == msg->Button1)
            {
                std::cout << "B1 has been pressed" << std::endl;
                //led_blink();
            }

            if (msg->button == msg->Button2)
            {
                std::cout << "B2 has been pressed" << std::endl;
		//sing_song();
            }
        }
    }
};

    //Function that sleeps. Used when playing sound
    void sleepok(int t, ros::NodeHandle &n) 
    {
	if (n.ok()) //Sleep for t seconds
	sleep(t);
    }

    //Turns the led's on or off
    bool led_on_off = false;

public:

    void led_blink(std_srvs::Trigger::Request &req,
                   std_srvs::Trigger::Response &res)
    {
	std::cout << "led_blink()" << std::endl;
       //Swaps the value of "led_on_off" between true and false
        if(led_on_off == false)
            led_on_off = true;
        else if(led_on_off == true)
            led_on_off = false;


        kobuki_msgs::Led msg1;
        kobuki_msgs::Led msg2;
	
        ros::Rate loop_rate(20);

	//While loop for blinking
        while (led_on_off)
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

    void move_square(std_srvs::Trigger::Request &req,
                     std_srvs::Trigger::Response &res)
    {
	std::cout << "move_square()" << std::endl;
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
    
    void sing_song(std_srvs::Trigger::Request &req,
                   std_srvs::Trigger::Response &res)
    {
	std::cout << "sing_song()" << std::endl;
    }

    void sing_song(){
	std::cout <<"singsongong" << std::endl;
	sleepok(1, n);
	//The path to the sound file
	const char *str = "/home/ros/megalovania.ogg";
	//Star the music
	sc.startWave(str);
	ROS_INFO("Playing music");
	sleepok(3, n);
    }

}
;
int main(int argc, char *argv[]){
    std::cout << "Use the buttons respectively to initiate the task" << std::endl;

    ros::init(argc, argv, "turtlebot");
    ros::NodeHandle nh;
    //Turtlebot Turtlebot_instance;

    //Jeg har kommenteret serviceserversne da de giver fejl lige nu :(((

    /*ros::ServiceServer server_square = nh.advertiseService("square", &Turtlebot::move_square, &Turtlebot_instance);
    ros::ServiceServer server_LED = nh.advertiseService("LED", &Turtlebot::led_blink, &Turtlebot_instance);
    ros::ServiceServer server_sing = nh.advertiseService("sing", &Turtlebot::sing_song, &Turtlebot_instance);*/
    
    //Initialize the class
    Turtlebot t;

    ros::spin();
    return 0;
    
}
