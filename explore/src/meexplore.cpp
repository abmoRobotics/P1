#include <explore/explore.h>
#include <std_srvs/Empty.h>

class dinmor{

  explore::Explore explorer;
public:

dinmor(){
  
}

void start(){
  explorer.start();
}

};
 
int main(int argc, char* argv[])
{
  ROS_INFO("guten tag");
  ros::init(argc, argv, "meexplore");
  dinmor e;
  ros::NodeHandle n;
  e.start();
  ros::spin();
  return 0;
} 