#include <ros/ros.h>
#include <main_pkg/routeName.h>
#include <cstdlib>
#include <string.h>



int main(int argc, char *argv[])
{
    std::string c;
    ros::init(argc,argv,"route_client"); 

    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<main_pkg::routeName>("route");

    main_pkg::routeName srv;
    

    std::cout << "Enter name: ";
    std::cin >> c;

    srv.request.name = c;



    if (client.call(srv)){
        std::cout << srv.response.result << std::endl;
    } else {
        ROS_INFO("FAILED SERVER");
        return 1;
    }


    

}