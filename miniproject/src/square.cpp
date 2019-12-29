#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>

class Classo{
public:
    Classo(){};
protected:
    enum{
        CORNER_0 = 0,
        CORNER_1 = 1,
        CORNER_2 = 2,
        CORNER_3 = 3,
    };
    bool rotate = false;
    bool move = false;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom", 1, &Classo::callback, this);
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
        double x2 =  std::abs(z-p1.pose.orientation.z);
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
            z =  0.7071;
            w = -0.7071;
            break;
        case CORNER_2:
            z =  0.0;
            w = -1.0;
            break;
        case CORNER_3:
            z = -0.7071;
            w = -0.7071;
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


/* 
         t.angular.z = 0.2;
        ros::Rate loop_rate(100);
        while(ros::ok() && angCost(presPoint,w,z) < 0.1){
            std::cout 
            << "Ang cost: " << angCost(presPoint,w,z) 
            << " z: " << presPoint.pose.orientation.z
            << " w: " << presPoint.pose.orientation.w 
            << std::endl;
            cmd_vel.publish(t);
            loop_rate.sleep();
        }
        t.angular.z = 0;

        t.linear.x = 0.2;
        while(ros::ok() && dist(presPoint,fromPoint) < 3){
            cmd_vel.publish(t);
            loop_rate.sleep();
        } */

        //corner++; 
        //moveSquare(corner);
    }

    void callback(const nav_msgs::Odometry::ConstPtr &message){
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
};

int main(int argc, char* argv[]){
    ros::init(argc,argv,"square");
    Classo c;
    ros::spin();
    return 0;
}
