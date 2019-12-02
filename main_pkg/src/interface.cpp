#include <iostream>
#include <ros/ros.h>
#include <main_pkg/routeName.h>
#include <main_pkg/serverMode.h>
#include <main_pkg/recieve_task_name.h>
#include <string>
#include <vector>
#include <std_srvs/Empty.h>

class Menu
{
private:
    ros::NodeHandle _nh;

    //Service clients
    ros::ServiceClient client_add_task = _nh.serviceClient<main_pkg::routeName>("add_task");
    ros::ServiceClient client_server_mode = _nh.serviceClient<main_pkg::serverMode>("server_mode");
    ros::ServiceClient client_stop_task = _nh.serviceClient<std_srvs::Empty>("stop_task");
    ros::ServiceClient client_recieve_task_name = _nh.serviceClient<main_pkg::recieve_task_name>("recieve_task_name");
    ros::ServiceClient client_turtlebot_job = _nh.serviceClient<main_pkg::serverMode>("turtlebot_job");
    ros::ServiceClient client_toggle_explore = _nh.serviceClient<std_srvs::SetBool>("toggle_explore");
    ros::ServiceClient client_show_maps = _nh.serviceClient<std_srvs::SetBool>("show_maps");

    //srv messages
    main_pkg::routeName srv_add_task;
    main_pkg::serverMode srv_server_mode;
    std_srvs::Empty srv_stop_task;
    std_srvs::Empty srv_show_maps;
    std_srvs::SetBool srv_toggle_explore;
    main_pkg::recieve_task_name srv_recieve_task_name;

public:
private:
    //Function that removes redundant information from console
    void _clearScreen() { std::cout << std::string(20, '\n'); }

    //Menu that shows when trying to create a route
        enum server_state
    {
        inactivate,
        taskCoordinates,
        kitchenPos,
        chargingPos
        //0 = inactivate no points can be stored
        //1 = points are stored to the task array
        //2 = points are stored to the kitchen position
    };
    void _createMenu()
    {
        _clearScreen();
        int selection = 0;
        std::string nameTask;
        std::cout << "-----------------------\nEnter name for task:" << std::endl;
        std::cin >> nameTask;
        //Function for sending name of task to server nameTask(nameTask);
        srv_add_task.request.name = nameTask;
        client_add_task.call(srv_add_task);
        //Function for changing server mode to allow for inserting points.
        srv_server_mode.request.mode = (int)taskCoordinates;
        client_server_mode.call(srv_server_mode);

        while (selection != 1)
        {
            _clearScreen();
            std::cout << " -----------------------\n1. Stop creating task and go back to menu" << std::endl;
            std::cout << "Enter: ";
            std::cin >> selection;
        }

        //Function for incremeting vector
        client_stop_task.call(srv_stop_task);
        //Changing server_mode to 0
        srv_server_mode.request.mode = (int)inactivate;
        client_server_mode.call(srv_server_mode);
    };

    void _automaticMapping()
    {
        char c;
        _clearScreen();
        //service start automatic mapping
        srv_toggle_explore.request.data = true;
        client_toggle_explore.call(srv_toggle_explore);
        std::cout << "Automatic mapping started" << std::endl;
        std::cout << "Press any key to return: ";
        std::cin >> c;
        srv_toggle_explore.request.data = false;
        client_toggle_explore.call(srv_toggle_explore);
    }

    void _saveMap(){
        cout << "Enter name for new map: ";
        string s;
        cin >> s;
        s = "rosrun map_server map_saver -f "+s;
        system(s);
    }

    void _showMaps()
    {
        cout << "Displaying list of maps. Enter the map number to load." << endl;
        client_show_maps.call(srv_show_maps);
        u_int mapNumber;
        cin >> mapNumber;
        string s = "rosrun map_server map_server ";
        s += ops[mapNumber];
        system(s);
        
    }

    void _sendTask()
    {
        char c;
        std::vector<float> k;
        // k = service request
        if (client_recieve_task_name.call(srv_recieve_task_name))
        {
            int length = srv_recieve_task_name.response.task_names.size();
            for (int i = 0; i < length; i++)
            {
                std::cout << i << ". " << srv_recieve_task_name.response.task_names[i] << std::endl;
            }
        }
        else
        {
            std::cout << "FAIL";
        }

        std::cin >> srv_server_mode.request.mode;
        client_turtlebot_job.call(srv_server_mode);
    }



    void _menu()
    {
        int c = 1;

        while (c != 0)
        {
            _clearScreen();
            std::cout << "----------------------------" << std::endl;
            std::cout << "1. Create route for Turtlebot" << std::endl;
            std::cout << "2. Send task for Turtlebot to perform" << std::endl;
            std::cout << "3. Start automatic mapping" << std::endl;
            std::cout << "4. Insert kitchen point" << std::endl;
            std::cout << "5. Insert charging station point" << std::endl;
            std::cout << "----------------------------" << std::endl;
            std::cout << "Select option: ";
            std::cin >> c;
            while (1 < c > 5)
            {
                switch (c){
                case 1:
                    _createMenu();
                    break;
                case 2:
                    _sendTask();
                    break;
                case 3:
                    _automaticMapping();
                    break;
                case 4:
                    _insert_kit();
                    break;
                case 5:
                    _automaticMapping();
                    break;

                default:
                    std::cout << "error" << std::endl;
                    break;
                }
            }
            
            
        }
    }

public:
    Menu()
    {
        _menu();
    }
    ~Menu() {}
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "interface");
    ros::NodeHandle nh;
    Menu e;
    //menu();
}