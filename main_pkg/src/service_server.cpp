#include <ros/ros.h>
#include <main_pkg/routeName.h>


bool result(main_pkg::routeName::Request &req,
            main_pkg::routeName::Response &res )
            {
                res.result = req.name + " BACK";
                ROS_INFO("Request with name ");
                ROS_INFO("RESPONSE: ");
                return 1;
            }


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "route_server");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("route", result);

    ROS_INFO("Server started");
    ros::spin();
    return 0;
}