#include <kobuki_msgs/SensorState.h>
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>

int kobuki_max_charge = 163; //check senere ved at checke volt når fuld, og indsæt selv eget tal.

void BatteryStateCallback(const kobuki_msgs::SensorState::ConstPtr &msg) //Callback når battery ændres
{
    float batterypct = float(msg->battery) / float(kobuki_max_charge) * 100; //Udregn % battery
    ROS_INFO("Volt: %d", msg->battery); //Output batteriet (i Volt)
    ROS_INFO("Battery: %i pct", int(round(batterypct))); //Output battery % (Rundet op og uden decimaler)
    
    ROS_INFO("dock status: %i", msg->charger);

    if (int (msg->charger) == 6){ //Hvis den er i docking
        ROS_INFO("Den er i opladeren");
    }
    else if(int ( msg->charger) == 0){ //Hvis output er 0
        ROS_INFO("Den er ikke i opladeren");
        //system("roslaunch kobuki_auto_docking minimal.launch --screen");
        //system("roslaunch kobuk i_auto_docking activate.launch --screen");
    }
    else if(int (msg->charger) == 2){
        ROS_INFO("Ladet op!");
    }

/*
    if (int (msg->charger) == 1 && batterypct > 97){
        // Påbegynd næste opgave hvis den er i docken og har en ladningsprocent højere end 97
        // Vi har valgt netop denne procent fordi lithium batterier ikke på have mere end 4,2 volt ladning
        // Og 97 = ((4*4,1) / (4*4.2) )* 100
    }
*/
}

int main(int argc, char **argv) //Main
{
    std::cout << "Checking Battery State" << std::endl;

    //Initiate ros
    ros::init(argc, argv, "kobuki_battery");

    ros::NodeHandle n;

    //Subscribe til battery
    ros::Subscriber sub = n.subscribe("/mobile_base/sensors/core", 1000, BatteryStateCallback);

    ros::spin();

    return 0;
}
