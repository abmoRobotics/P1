#include <algorithm>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <vector>
#include <main_pkg/pose.h>
#include <main_pkg/poseArray.h>
#include <main_pkg/poseTasks.h>
#include <main_pkg/serverMode.h>
#include <main_pkg/routeName.h>
#include <main_pkg/recieve_task_name.h>
#include <string.h>
#include <std_srvs/Empty.h>


void menu();
void point();
void choose_task();

int i = 0;
int j = 0;

struct work_tasks
{
    std::string name;
    geometry_msgs::PoseArray points;
};

work_tasks duties;
std::vector<work_tasks> tasks;


work_tasks duties2;
std::vector<work_tasks> turtlebot_points;




geometry_msgs::PointStamped pose_kitchen;

int server_mode = 0; //1 = insert points, 2 = insert kitchen coordinate:

void insert_point(const geometry_msgs::PointStamped::ConstPtr msg)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = msg->point.x;
    pose.pose.position.y = msg->point.y;
    pose.pose.position.z = msg->point.z;
    pose.pose.orientation.w = 1.0;
    duties.points.header.stamp = ros::Time::now();
    duties.points.header.frame_id = i;
    duties.points.poses.push_back(pose.pose);

    std::cout << "punkt: " << duties.points.poses[i] << std::endl;
    i++;
    /*
    if ((c == 2)||(c==3))
    {

        tasks.push_back(duties);
        j++;
        duties.points.poses.clear();
        duties.name = "";
        std::cout << "POINTS: " << duties.points.poses[i] << std::endl;
        std::cout << "TASKS: " << tasks[0].points.poses[0] << std::endl;
        i = 0;
    }
    if (c == 3)
    {
        menu();
    }*/
}

void insert_kitchen(const geometry_msgs::PointStamped::ConstPtr msg){
    pose_kitchen.point.x = msg->point.x;
    pose_kitchen.point.y = msg->point.y;
    pose_kitchen.point.z = msg->point.z;
    pose_kitchen.header.stamp = ros::Time::now();
    pose_kitchen.header.frame_id = 1;
}

void recieve_points(const geometry_msgs::PointStamped::ConstPtr &msg){
    switch (server_mode)
    {
    case 1: //Insert point in task
        insert_point(msg);
        break;
    case 2:
        insert_kitchen(msg);
        break;
    default:
        ROS_INFO("SERVER: CANNOT INSERT POINT");
        break;
    }
}

bool change_server_mode(main_pkg::serverMode::Request &req,
                        main_pkg::serverMode::Response &res){
                            server_mode = req.mode;
                            res.response = server_mode;
                            ROS_INFO("Changed mode");
                            return 1;
                        }

bool add_task(main_pkg::routeName::Request &req,
              main_pkg::routeName::Response &res){
                   duties.name = req.name;
                   return 1;     
              }

bool stop_task(std_srvs::Empty::Request &req,
              std_srvs::Empty::Response &res){
    
    tasks.push_back(duties);
    duties.points.poses.clear();

}

bool send_task_name(main_pkg::recieve_task_name::Request &req,
                        main_pkg::recieve_task_name::Response &res){
                            
                            for(int i = 0; i < tasks.size(); i++){
                                res.task_names.push_back(tasks[i].name);
                                
                                
                            }
                            return 1;

                        }

bool turtlebot_job(main_pkg::serverMode::Request &req,
                   main_pkg::serverMode::Response &res){
                        int i = req.mode;
                        turtlebot_points.push_back(tasks[i]);

                        for (size_t i = 0; i < turtlebot_points.size(); i++)
                            {
                                std::cout << turtlebot_points.size() << std::endl;
                                std::cout << i << ". Name: " << turtlebot_points[i].name << std::endl;
                            }
                        }



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Caterroute");
    ros::NodeHandle nh;

    ros::Rate loop_rate(1);

    ros::Subscriber click_sub = nh.subscribe("clicked_point", 100, &recieve_points);

    ros::ServiceServer server = nh.advertiseService("add_task", add_task);
    ros::ServiceServer server2 = nh.advertiseService("stop_task", stop_task);
    ros::ServiceServer server3 = nh.advertiseService("server_mode", change_server_mode);
    ros::ServiceServer server4 = nh.advertiseService("recieve_task_name", send_task_name);
    ros::ServiceServer server5 = nh.advertiseService("turtlebot_job", turtlebot_job);
    


    while (ros::ok())
    {
        for (size_t i = 0; i < tasks.size(); i++)
        {
            std::cout << i << ". Name: " << tasks[i].name << std::endl;
        }
        
        ros::spinOnce();
        loop_rate.sleep();


    }

    ros::spin();
}