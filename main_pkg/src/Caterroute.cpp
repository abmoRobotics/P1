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
#include <string.h>

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

std::vector<geometry_msgs::PoseArray> pub_tasks;

main_pkg::pose data;
main_pkg::poseArray dataArray;
main_pkg::poseTasks dataTasks;

void point(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    int c;

    if (duties.name.empty())
    {
        std::cout << "Enter name of task: ";
        std::cin >> duties.name;
    }

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

    while ((c != 1) && (c != 2) && (c != 3))
    {
        std::cout << "1. To insert more points" << std::endl;
        std::cout << "2. Create new task" << std::endl;
        std::cout << "3. Back to menu" << std::endl;
        std::cin >> c;
    }

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
    }
}

void menu()
{
    int c;
    std::cout << "1. create a routee" << std::endl;
    std::cout << "2. Initiate automatic mapping" << std::endl;
    std::cout << "3. Choose task for robot" << std::endl;
    std::cin >> c;

    if (c == 2)
    {
    } //send besked
    else if (c == 3)
    {
        choose_task();
    }
}

void choose_task()
{
    char c;

    do{
        int choice;
        for (i = 0; i < tasks.size(); i++)
        {
            std::cout << i << ". " << tasks[i].name << std::endl;
        }
        std::cin >> choice;

        for (int o = 0; o < tasks[choice].points.poses.size(); o++)
        {
            data.x = tasks[choice].points.poses[o].position.x;
            data.y = tasks[choice].points.poses[o].position.y;
            data.z = tasks[choice].points.poses[o].position.z;
            dataArray.poseArray.push_back(data);
        }
        dataTasks.poseTasks.push_back(dataArray);
        dataArray.poseArray.clear();
        std::cout << "Do you want to add another task to the queue (y/n)";
        std::cin >> c;
    } while (c =='y');

    menu();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Caterroute");
    ros::NodeHandle nh;

    ros::Subscriber click_sub;

    ros::Rate loop_rate(1);
    ros::Publisher tasks_pub = nh.advertise<main_pkg::poseTasks>("/tasks", 1000);

    click_sub = nh.subscribe("clicked_point", 100, &point);

    while (ros::ok())
    {

        /*
        for (int p = 0; p < tasks.size(); p++){

                for (int o = 0; o < tasks[p].points.poses.size(); o++){
                    data.x = tasks[p].points.poses[o].position.x;
                    data.y = tasks[p].points.poses[o].position.y;
                    data.z = tasks[p].points.poses[o].position.z;
                    dataArray.poseArray.push_back(data);
                }
                dataTasks.poseTasks.push_back(dataArray);
                dataArray.poseArray.clear();
        }*/

        tasks_pub.publish(dataTasks);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
}