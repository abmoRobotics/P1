#include <ros/ros.h>
#include <main_pkg/poseArray_srv.h>
#include <geometry_msgs/PoseArray.h>


class Movement{
    public:
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<main_pkg::poseArray_srv>("get_job");

    
    geometry_msgs::PoseArray goal;

    bool _request_job(){
            main_pkg::poseArray_srv srv;
            client.call(srv);
            goal = srv.response.arr;
            for(int i=0; i<srv.response.arr.poses.size(); i++){
                std::cout << srv.response.arr.poses[i].position << std::endl;
        }
    }

    
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "caterbot");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<main_pkg::poseArray_srv>("get_job");
    main_pkg::poseArray_srv srv;
    client.call(srv);
    for(int i=0; i<srv.response.arr.poses.size(); i++){
        std::cout << srv.response.arr.poses[i].position << std::endl;
    }



}
