#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/BumperEvent.h>

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        sleep(t);
}

void ButtonCallback(const kobuki_msgs::ButtonEvent::ConstPtr &msg)
{
    if (msg->state == msg->PRESSED)
    {
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
        }
    }
}

void BumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    if (msg->state == msg->PRESSED)
    {
        ros::NodeHandle nh;
        sound_play::SoundClient sc;

        sleepok(1, nh);
	const char *str = "/home/ros/megalovania.ogg";
        sc.startWave(str);
        ROS_INFO("Spiller megalovania");
	sleepok(3, nh);
	sc.stopWave(str);
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

    // while(nh.ok())
    // {
    //     sc.playWave("/home/ros/megalovania.ogg");
    //     sleepok(2, nh);
    // }

    ros::spin();

    return 0;
}
