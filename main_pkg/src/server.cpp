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
#include <main_pkg/poseArray.h>
#include <main_pkg/poseTasks.h>
#include <main_pkg/pointStamped_srv.h>
#include <main_pkg/serverMode.h>
#include <main_pkg/routeName.h>
#include <main_pkg/recieve_task_name.h>
#include <main_pkg/poseArray_srv.h>
#include <string.h>
#include <std_srvs/Empty.h>

class Server
{
    int i = 0;

    //Stucture for storing tasks "name" is for giving the task a name,
    // and PoseArray is an array consisting of coordinates.
public:
    struct store_task
    {
        std::string name;
        geometry_msgs::PoseArray poseArray;
    };

    //Two instances of store_task is defined
    store_task savedTasks;     //for saving the different tasks
    store_task publishedTasks; //For publishing the tasks for the robot to perform.

    //Each of these instances of store_task needs to be stored within a vector.
public:
    std::vector<store_task> v_savedTasks;
    std::vector<store_task> v_publishedTasks;

    //We also define a point for the kitchen
    geometry_msgs::PointStamped pose_kitchen;

private:
    //We define the different state of the server as an enum
    enum server_state
    {
        inactivate,
        storeTaskCoordinates,
        storeKitchenPosition
        //0 = inactivate no points can be stored
        //1 = points are stored to the task array
        //2 = points are stored to the kitchen position
    };
    int server_mode = 0;


    void insert_point(const geometry_msgs::PointStamped::ConstPtr msg)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = msg->point.x;
        pose.pose.position.y = msg->point.y;
        pose.pose.position.z = msg->point.z;
        pose.pose.orientation.w = 1.0;
        savedTasks.poseArray.header.stamp = ros::Time::now();
        savedTasks.poseArray.header.frame_id = msg->header.frame_id;
        savedTasks.poseArray.poses.push_back(pose.pose);

        std::cout << "punkt: " << savedTasks.poseArray.poses[i] << std::endl;
        i++;
    }

    void insert_kitchen(const geometry_msgs::PointStamped::ConstPtr msg)
    {
        pose_kitchen.point.x = msg->point.x;
        pose_kitchen.point.y = msg->point.y;
        pose_kitchen.point.z = msg->point.z;
        pose_kitchen.header.stamp = ros::Time::now();
        pose_kitchen.header.frame_id = msg->header.frame_id;
    }

public:
    void recieve_points(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
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
                            main_pkg::serverMode::Response &res)
    {
        server_mode = req.mode;
        res.response = server_mode;
        ROS_INFO("Changed mode");
        return 1;
    }

    bool add_task(main_pkg::routeName::Request &req,
                  main_pkg::routeName::Response &res)
    {
        savedTasks.name = req.name;
        return 1;
    }

    bool stop_task(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res)
    {
        v_savedTasks.push_back(savedTasks);
        savedTasks.poseArray.poses.clear();
    }

    bool send_task_name(main_pkg::recieve_task_name::Request &req,
                        main_pkg::recieve_task_name::Response &res)
    {
        for (int i = 0; i < v_savedTasks.size(); i++)
        {
            res.task_names.push_back(v_savedTasks[i].name);
        }
        return 1;
    }

    bool turtlebot_job(main_pkg::serverMode::Request &req,
                       main_pkg::serverMode::Response &res)
    {
        v_publishedTasks.push_back(v_savedTasks[req.mode]);

        for (size_t i = 0; i < v_publishedTasks.size(); i++)
        {
            std::cout << v_publishedTasks.size() << std::endl;
            std::cout << i << ". Name: " << v_publishedTasks[i].name << std::endl;
        }
    }

    bool get_job(main_pkg::poseArray_srv::Request &req,
                 main_pkg::poseArray_srv::Response &res)
    {
        if (!v_publishedTasks.empty())
        {
            res.arr = v_publishedTasks[0].poseArray;
            v_publishedTasks.erase(v_publishedTasks.begin());
        }
        else
        {
            ROS_INFO("NO TASKS");
        }
    }

    bool get_pose_kitchen(main_pkg::pointStamped_srv::Request &req,
                 main_pkg::pointStamped_srv::Response &res)
    {
	//Check if pose_kitchen has been set
        if(pose_kitchen.point.x != 0 && pose_kitchen.point.y != 0 && pose_kitchen.point.z != 0){
	    ROS_INFO("Kitchen point found!");	
	    res.pose = pose_kitchen;
        }
	else{		
	    ROS_INFO("There is no point set for the kitchen");		
	}

    }

public:
    Server() {}
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Caterroute");
    ros::NodeHandle nh;

    Server server_instance;
    //Subscribers
    ros::Subscriber click_sub = nh.subscribe("clicked_point", 100, &Server::recieve_points, &server_instance);
    //service servers
    ros::ServiceServer server = nh.advertiseService("add_task", &Server::add_task, &server_instance);
    ros::ServiceServer server2 = nh.advertiseService("stop_task", &Server::stop_task, &server_instance);
    ros::ServiceServer server3 = nh.advertiseService("server_mode", &Server::change_server_mode, &server_instance);
    ros::ServiceServer server4 = nh.advertiseService("recieve_task_name", &Server::send_task_name, &server_instance);
    ros::ServiceServer server5 = nh.advertiseService("turtlebot_job", &Server::turtlebot_job, &server_instance);
    ros::ServiceServer server6 = nh.advertiseService("get_job", &Server::get_job, &server_instance);
    ros::ServiceServer server7 = nh.advertiseService("get_pose_kitchen", &Server::get_pose_kitchen, &server_instance);
    //Publisher

    geometry_msgs::PoseArray msg_pose;

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        if (!server_instance.v_publishedTasks.empty())
        {
            std::cout << "PENDING TASKS:" << std::endl;
            for (size_t i = 0; i < server_instance.v_publishedTasks.size(); i++)
            {
                std::cout << "TASK " << i + 1 << ": " << server_instance.v_publishedTasks[i].name << std::endl;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
}
