
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