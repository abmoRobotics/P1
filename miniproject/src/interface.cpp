#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

class menu
{ 
private:
//Service
 ros::NodeHandle n;

 ros::ServiceClient client_square = n.serviceClient<std_srvs::Trigger>("square");
 ros::ServiceClient client_LED = n.serviceClient<std_srvs::Trigger>("LED");
 ros::ServiceClient client_sing = n.serviceClient<std_srvs::Trigger>("sing");


 std_srvs::Trigger square;
 std_srvs::Trigger LED;
 std_srvs::Trigger sing;

public:

 void makeasquare()
 {
   if (client_square.call(square))
   {
       ROS_INFO("Task received");
   }
   else
   {
       ROS_ERROR("Service was not called");
   }
          
 };


 void startLED()
 {
    if (client_LED.call(LED))
    {
        ROS_INFO("Task received");
    }
    else
    {
        ROS_ERROR("Service was not called");
    }
 
 };

 void robotsing()
 {
    if (client_sing.call(sing))
    {
       ROS_INFO("Task received"); 
    }
    else
    {
        ROS_ERROR("Service was not called");
    }
       
 };



  void Menu()
 {
    int c=1;

    while(c !=0)
    {
        system("clear");
        std::cout << "1. Make robot move in a square" << std::endl;
        std::cout << "2. Make LED light up" << std::endl;
        std::cout << "3. Make robot sing" << std::endl;
        std::cout << "-----------------------------" << std::endl;
        std::cout << "Select an option" << std::endl;

        std::cin >> c;

        while (1>c>3)
        {
            std::cin >> c;
        }
        
        switch (c)
        {
        case 1:
            makeasquare();
            break;
        case 2:
            startLED();
            break;
        case 3:
            robotsing();
            break;
        default:
            
            break;
        }
    }
 }
   menu()
  {
      Menu();
  }

};

int main(int argc, char *argv[])
{
   ros::init(argc, argv, "interface");
   menu M;
   ros::spin();
   
}