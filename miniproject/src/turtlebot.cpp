#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
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
    //SoundClient
    sound_play::SoundClient sc; 

    //Turns the led's on or off
    bool led_on_off = false;

public:

    bool blinking = false;
    bool led_blink(std_srvs::Trigger::Request &req,
                   std_srvs::Trigger::Response &res)
    {
        blinking = !blinking;
    }

    void blink(){

        kobuki_msgs::Led msg1;
        kobuki_msgs::Led msg2;
	

	//While loop for blinking
        while (ros::ok && blinking)
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
            std::cout << "blinked" << std::endl;
            ros::spinOnce();
        }
         
        
    }
    
    bool sing_song(std_srvs::Trigger::Request &req,
                   std_srvs::Trigger::Response &res){
    static bool playing = false;
	std::cout <<"Sing song" << std::endl;
	//Start the music
    if(!playing){
        sc.startWaveFromPkg("miniproject", "sound/megalovania.ogg");
        ROS_INFO("Playing music");
        playing = !playing;
    }
    //Stop the music
    else if(playing){
        sc.stopWaveFromPkg("miniproject", "sound/megalovania.ogg");
        ROS_INFO("Stopped music");
        playing = !playing;
    }
	//sleepok(3, n);
    }
    public:
    Turtlebot(){
        blink();
    }

};

class Square{
public:
    
    bool toggle(std_srvs::Trigger::Request &req,
                std_srvs::Trigger::Response &res){
        start = !start;
        return 1;
    }
protected:
    enum{
        CORNER_0 = 0,
        CORNER_1 = 1,
        CORNER_2 = 2,
        CORNER_3 = 3,
    };
    bool rotate = false;
    bool move = false;
    bool start = false;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom", 1, &Square::callback, this);
    ros::Publisher cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
    geometry_msgs::PoseStamped presentPoint ,fromPoint;


    double dist(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2){
        return std::sqrt(
            std::pow(p1.pose.position.x-p2.pose.position.x,2)+
            std::pow(p1.pose.position.y-p2.pose.position.y,2)
            );
    }

    double angleCost(geometry_msgs::PoseStamped p1, double w, double z){
        double x1 =  std::abs(w-p1.pose.orientation.w);
        std::cout << "w: " << w << "p1w: " << p1.pose.orientation.w << std::endl;
        double x2 =  std::abs(z-p1.pose.orientation.z);
        std::cout << "z: " << z << "p1z: " << p1.pose.orientation.z << std::endl;
        return x1+x2;
    }
    
    void  moveSquare(){
        static int corner = CORNER_0;
        double w, z;
        switch (corner)
        {
        case CORNER_0:
            z = 1.0;
            w = 0.0;
            break;
        case CORNER_1:
            z =  -0.7071;
            w = 0.7071;
            break;
        case CORNER_2:
            z =  0.0;
            w = 1.0;
            break;
        case CORNER_3:
            z = 0.7071;
            w = 0.7071;
            break;
        default:
            corner = CORNER_0;
            return;
        };

        geometry_msgs::Twist t;
        if(rotate && move) move = false;
        if(rotate){
            if (angleCost(presentPoint,w,z) < 0.05){
                fromPoint.pose.position.x = presentPoint.pose.position.x;
                fromPoint.pose.position.y = presentPoint.pose.position.y;
                corner++;
                rotate = false;
                move = true;
            }else{
                t.angular.z = 0.4;
                std::cout << angleCost(presentPoint, w, z) << std::endl;
            }
        }else if(move){
            if(dist(presentPoint,fromPoint) > 1){
                move = false;
                rotate = true;
            }else{
                t.linear.x = 0.4;
            }
        }else{
            rotate = true;
        }
        cmd_vel.publish(t);
    }

    void callback(const nav_msgs::Odometry::ConstPtr &message){
        if(!start)return;
        presentPoint.pose.orientation.z = message->pose.pose.orientation.z;
        presentPoint.pose.orientation.w = message->pose.pose.orientation.w;

        presentPoint.pose.position.x = message->pose.pose.position.x;
        presentPoint.pose.position.y = message->pose.pose.position.y; 
        std::cout << "callback received"<<
        " z: " << presentPoint.pose.orientation.z <<
        " w: " << presentPoint.pose.orientation.w <<
        " x: " << presentPoint.pose.orientation.x <<
        " y: " << presentPoint.pose.orientation.y <<
        " distance: " << dist(presentPoint,fromPoint) <<
        std::endl;
        moveSquare();
    }
    public:
    Square(){

    };
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "turtlebot");
    ros::NodeHandle nh;
    Turtlebot Turtlebot_instance;
    Square Square_instance;

    ros::ServiceServer server_square = nh.advertiseService("square", &Square::toggle, &Square_instance);
    ros::ServiceServer server_LED = nh.advertiseService("LED", &Turtlebot::led_blink, &Turtlebot_instance);
    ros::ServiceServer server_sing = nh.advertiseService("sing", &Turtlebot::sing_song, &Turtlebot_instance);

    ros::Rate loop_rate(20);
    while (ros::ok)
    {
        Turtlebot_instance.blink();
        ros::spinOnce();
        loop_rate.sleep();
    }
    


    ros::spin();
    return 0;
    
}