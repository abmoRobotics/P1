#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
//5. navngiv callback funktionen. den skal returnere ingenting og 
//parameteren skal være: const sensor_msgs::LaserScanConstPtr &message
void callback(const sensor_msgs::LaserScanConstPtr &message){
    //6. initialiser en variabel, som holder på komma-tal og 
    //giv den navnet "lavest". Få den til at starte med at være værdien 10000:
    double lavest = 10000;
    //7. lav en for -lykke, hvor man, inde i scan-beskeden, itererer 
    //igennem hele mængden af "ranges". "ranges" ligger i "message" og tilgås 
    //ved en pil( -> ). Størrelsen af mængden af "range" listen er: ranges.size()
    for(int i = 0; i < message->ranges.size();i++){
        //8. hvis denne range[i] er mindre end den hidtil laveste, skal 
        //"lavest" få værdien: range[i]. (indeholder if-statement)
        if(message->ranges[i] < lavest){
            lavest = message->ranges[i];
        }
    }
    //9. Print den mindste afstand ud til terminalen med "std::to_string()" og "ROS_INFO_STREAM"
    std::cout << "Distance is " << std::to_string(lavest) << " metres"<<std::endl;
}


int main(int argc, char *argv[]){

    
    //1. initialiser ros for denne node. parametrene skal være: argc, argv og ">>fil navnet<<":
    ros::init(argc,argv,"range_check");
    //2. initialiser en nodehandle instance
    ros::NodeHandle n;

    
    //3. subcribe til "/scan" topic, konstant lagre 1 besked i hukommelsen, samt 
    //angiv en callback funktion til at behandle beskederne fra "/scan"(der skal 
    //være et "&"foran callbackens navn):
    ros::Subscriber sub = n.subscribe("/scan",1,&callback);
    //4. få ros til at spinne (ikke kun få én besked):
    ros::spin();


    return 1;
}
