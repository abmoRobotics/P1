# P1-Projekt

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

//5. navngiv callback funktionen. den skal returnere ingenting og parameteren skal være: const sensor_msgs::LaserScanConstPtr &message

    //6. initialiser en komma variabel og få den til at starte med at være en højt tal:
    
    //7. lav en for -lykke, hvor man, inde i scan-beskeden, itererer igennem hele mængden af "ranges" (ranges.size())
    for(){
        //8. hvis denne range er mindre end den hidtil laveste, skal den laveste sættes til denn range:
    }
    //9. Print den mindste afstand ud til terminalen med "std::to_string()" og "ROS_INFO_STREAM".

}

int main(int argc, char *argv[]){

    //1. initialiser ros for denne node:
    
    //2. initialiser en nodehandle instance
    
    //3. subcribe til "/scan" topic samt angiv en callback funktion til at behandle beskederne fra "/scan":
    
    //4. få ros til at spinne (ikke kun få én besked):
    


    return 1;
}
