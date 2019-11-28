#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Twist.h>

bool sans = false;

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        sleep(t);
}

void ButtonCallback(const kobuki_msgs::ButtonEvent::ConstPtr &msg)
{
    if (msg->state == msg->PRESSED)
	sans = true;
    /*{
        if (msg->button == msg->Button0)
        {
            ros::NodeHandle nh;
            sound_play::SoundClient sc;

            sleepok(1, nh);
            sc.say("Hello world my name is turtlebot");
            ROS_INFO("Speaking");
            sleepok(2, nh);
        }
        else if (msg->button == msg->Button1)
        {
            ros::NodeHandle nh;
            sound_play::SoundClient sc;

            sleepok(1, nh);
            sc.say("Reeeeeeeeeeeeeeeeee");
            ROS_INFO("Speaking");
            sleepok(2, nh);
        }
        else if (msg->button == msg->Button2)
        {
            ros::NodeHandle nh;
            sound_play::SoundClient sc;

            sleepok(1, nh);
            sc.say("hands up motherfucker you are under arrest");
            ROS_INFO("Speaking");
            sleepok(2, nh);
        }*/
    //}
}

void BumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    if ((msg->state == msg->PRESSED) && sans == false)
    {
	sans = true;
    //MUSIC //

	ros::NodeHandle nh;
	sound_play::SoundClient sc;

	sleepok(1, nh);
	const char *str = "/home/ros/megalovania.ogg";
	sc.startWave(str);
	ROS_INFO("Spiller megalovania");
	sleepok(3, nh);
	//sc.stopWave(str);

    // SPIN //
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

    geometry_msgs::Twist cmd_vel_message;
    cmd_vel_message.angular.z = 1.0;
    cmd_vel_message.linear.x = 0;

    //Current time
    double begin = ros::Time::now().toSec();
    double goal = begin + 13.0;

	ROS_INFO("begin: %f", begin);
	ROS_INFO("goal: %f", goal);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
	ROS_INFO("current time: %f", ros::Time::now().toSec());

	//double

	if(ros::Time::now().toSec() >= goal)
		cmd_vel_message.angular.z = 3.0;
	
	cmd_vel_pub.publish(cmd_vel_message);

	loop_rate.sleep();
    }


    }
}

// //Main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sound_test");

    ROS_INFO("Starter lyd!");


    ros::NodeHandle buttonNode;

    ros::Subscriber button_sub = buttonNode.subscribe("/mobile_base/events/button", 1000, ButtonCallback);

    ros::NodeHandle bumperNode;

    ros::Subscriber bumper_sub = bumperNode.subscribe("/mobile_base/events/bumper", 1000, BumperCallback);

	ros::spin();

    return 0;
}
