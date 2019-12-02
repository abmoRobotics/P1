#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

//5. navngiv callback funktionen. den skal returnere ingenting og parameteren skal være: const sensor_msgs::LaserScanConstPtr &message
void callback(const sensor_msgs::LaserScanConstPtr &message){
    //6. initialiser en komma variabel og få den til at starte med at være en højt tal:
    double lowest = 10000;
    //7. lav en for -lykke, hvor man, inde i scan-beskeden, itererer igennem hele mængden af "ranges" (ranges.size())
    for(int i = 0; i < message->ranges.size();i++){
        //8. hvis denne range er mindre end den hidtil laveste, skal den laveste sættes til denn range:
        if(message->ranges[i] < lowest){
            lowest = message->ranges[i];
        }
    }
    //9. Print den mindste afstand ud til terminalen med "std::to_string()" og "ROS_INFO_STREAM"
    std::string s = "Distance is "+ std::to_string(lowest) + "metres";
    ROS_INFO_STREAM(s);
}

int main(int argc, char *argv[]){

    //1. initialiser ros for denne node:
    ros::init(argc,argv,"range_check");
    //2. initialiser en nodehandle instance
    ros::NodeHandle nh;
    //3. subcribe til "/scan" topic samt angiv en callback funktion til at behandle beskederne fra "/scan":
    ros::Subscriber sub = nh.subscribe("/scan",1, &callback);
    //4. få ros til at spinne (ikke kun få én besked):
    ros::spin();

    return 1;
}