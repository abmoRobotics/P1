# P1-Projekt



#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

class Move{
    //Actions is defined.
    ros::NodeHandle n;
    ros::Publisher pub;
    
    public:
    Move() : MoveBaseClient("move_base",true){
        pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal" ,1 );
        xPoint.point.x = 15;
        xPoint.point.y = 15;
    }
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    geometry_msgs::PointStamped xPoint, currentPos;
    //tf::TransformListener listener;

    
    void move_base(move_base_msgs::MoveBaseGoal goal){
        std::cout << "Waiting for server. ";
        MoveBaseClient.waitForServer();            
        std::cout << " Sending goal. ";
        MoveBaseClient.sendGoal(goal);             
        std::cout << " Waiting for result. "<<std::endl;
        MoveBaseClient.waitForResult();            
        std::cout << " Result received: ";
	if(this->MoveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        std::cout << "Success." << std::endl;
	}else{
        std::cout << "Failiure." << std::endl;
	}

    }

    bool setPoint(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res)
    {
        std::cout << "setting point." << std::endl;
        xPoint.point = currentPos.point;
        std::cout << "x: " << xPoint.point.x << " y: " << xPoint.point.y;
        xPoint.header.frame_id = currentPos.header.frame_id;
        std::cout << " frame id: " << xPoint.header.frame_id <<std::endl;
        xPoint.header.stamp = ros::Time::now();
    }

    bool returnPoint42(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res)
    {
        geometry_msgs::PointStamped p;
        //listener.transformPoint("map",xPoint,p);
        std::cout << "returning to point 42." << std::endl;
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time();
        std::string frame = "/map";
        msg.header.frame_id = frame.c_str();
        msg.pose.position = p.point;
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        std::cout << "Moving to: x: " << xPoint.point.x << " " << p.point.x << 
        " y: " << xPoint.point.y << " " << p.point.y <<std::endl;
        pub.publish(msg);
        
    }


    bool returnPoint(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res)
    {
        std::cout << "returning to point." << std::endl;
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = xPoint.point.x;
        goal.target_pose.pose.position.y = xPoint.point.y;
	    goal.target_pose.pose.orientation.w = 1.0;
        std::cout << "Moving to: x: " << xPoint.point.x << " y: " << xPoint.point.y <<std::endl;
        move_base(goal);
    }

    void position(const nav_msgs::Odometry::ConstPtr &msg)
    {
            //save position 
	currentPos.point = msg->pose.pose.position;
	currentPos.header.frame_id = msg->header.frame_id;
	currentPos.header.stamp = ros::Time::now();
    
    }

};
// This is where we start
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "move_serv");
    Move m;
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("set_point", &Move::setPoint, &m);
    //ros::ServiceServer server1 = nh.advertiseService("return_point", &Move::returnPoint, &m);
    ros::ServiceServer server2 = nh.advertiseService("return_point", &Move::returnPoint42, &m);
    ros::Subscriber sub = nh.subscribe("/odom", 1, &Move::position, &m);
    ros::spin();
    return 0;
}


#include <ros/ros.h>
#include <std_srvs/Empty.h>

// This is where we start
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "move_client");
    ros::NodeHandle nh;
    ros::ServiceClient client_set_point = nh.serviceClient<std_srvs::Empty>("set_point");
    ros::ServiceClient client_return_point = nh.serviceClient<std_srvs::Empty>("return_point");
    std_srvs::Empty srv_set_point;
    std_srvs::Empty srv_return_point;

    int i;
    while(true){
        std::cout << "1 to save point. 2 to return. 7 to exit: "<<std::endl;
        std::cin >> i;
	if(i == 7){return 42;}
        if(i == 1){
     	    std::cout << "Setting point." << std::endl;
            client_set_point.call(srv_set_point);
        }else{
	    std::cout << "Returning to point." << std::endl;
	    client_return_point.call(srv_return_point);
	}

    }

    return 0;
}
 
