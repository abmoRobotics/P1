#include <kobuki_msgs/SensorState.h>
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>

int kobuki_max_charge = 168; //check senere ved at checke volt når fuld, og indsæt selv eget tal.

void BatteryStateCallback(const kobuki_msgs::SensorState::ConstPtr &msg)
{
    float batterypct = float(msg->battery) / float(kobuki_max_charge) * 100;
    ROS_INFO("Volt: %d", msg->battery);
    ROS_INFO("Battery: %i pct", int(round(batterypct)));
    
    if (int (msg->charger) == 1){
        ROS_INFO("Den er i opladeren");
    }
    else {
        ROS_INFO("Den er ikke i opladeren");
    }
/*
    if (int (msg->charger) == 1 && batterypct > 97){
        // Påbegynd næste opgave hvis den er i docken og har en ladningsprocent højere end 97
        // Vi har valgt netop denne procent fordi lithium batterier ikke på have mere end 4,2 volt ladning
        // Og 97 = ((4*4,1) / (4*4.2) )* 100
    }
*/
}

int main(int argc, char **argv)
{
    std::cout << "Checking Battery State" << std::endl;

    ros::init(argc, argv, "kobuki_battery");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/mobile_base/sensors/core", 1000, BatteryStateCallback);

    ros::spin();

    return 0;
}
