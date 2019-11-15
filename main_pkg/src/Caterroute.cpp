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


int i = 0;
int j = 0;
geometry_msgs::PoseArray points;
std::vector<geometry_msgs::PoseArray> tasks;




void point(const geometry_msgs::PointStamped::ConstPtr& msg){
    char c;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = msg->point.x;
    pose.pose.position.y = msg->point.y;
    pose.pose.position.z = msg->point.z;
    pose.pose.orientation.w = 1.0;
    points.header.stamp = ros::Time::now();
    points.header.frame_id = i;
    points.poses.push_back(pose.pose);

    std::cout << "punkt: " << points.poses[i] << std::endl;
    i++;
    std::cout << "Vil du indsætte flere punkter?(y/n): ";
    std::cin >> c;
    if (c == 'n'){
        tasks.push_back(points);
        j++;
        points.poses.clear();
        std::cout << "POINTS: " << points.poses[i] << std::endl;
        std::cout << "TASKS: " << tasks[0].poses[0] << std::endl;
        i=0;
    }
}

int main(int argc, char *argv[]){
    ros::init(argc,argv, "Caterroute");
    ros::NodeHandle nh;
    

    
    ros::Subscriber click_sub;

    ros::Rate loop_rate(1);
    ros::Publisher tasks_pub = nh.advertise<main_pkg::poseTasks>("/tasks", 1000);
    click_sub = nh.subscribe("clicked_point", 100, &point);

    while (ros::ok()){
        main_pkg::pose data;
        main_pkg::poseArray dataArray;
        main_pkg::poseTasks dataTasks;
        
        
        for (int p = 0; p < tasks.size(); p++){

                for (int o = 0; o < tasks[p].poses.size(); o++){
                    std::cout << tasks[o].poses.size() << std::endl;
                    data.x = tasks[p].poses[o].position.x;
                    data.y = tasks[p].poses[o].position.y;
                    data.z = tasks[p].poses[o].position.z;
                    dataArray.poseArray.push_back(data);
                }
                dataTasks.poseTasks.push_back(dataArray);
                dataArray.poseArray.clear();
            
        }

        /*
        for (int p = 0;p < 3;p++){
            data.x = 1.0+p;
            data.y = 2.0+p;
            data.z = 3.0+p;
            dataArray.poseArray.push_back(data);
        }*/

        
        tasks_pub.publish(dataTasks);
        ros::spinOnce();
        loop_rate.sleep();
    }
    


    
    
    
    ros::spin();
}