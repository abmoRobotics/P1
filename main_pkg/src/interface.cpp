#include <iostream>
#include <ros/ros.h>
#include <main_pkg/routeName.h>
#include <main_pkg/serverMode.h>
#include <main_pkg/recieve_task_name.h>
#include <main_pkg/navMode.h>
#include <string>
#include <vector>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <actionlib/client/simple_action_client.h>
#include <main_pkg/reverseAction.h>
#include <std_srvs/SetBool.h>
#include <sstream>
#include <kobuki_msgs/SensorState.h>

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
    ros::ServiceClient client_change_navMode = _nh.serviceClient<main_pkg::navMode>("change_navMode");
    ros::ServiceClient client_return_to_dock = _nh.serviceClient<std_srvs::SetBool>("return_to_dock");


    //srv messages
    main_pkg::routeName srv_add_task;
    main_pkg::serverMode srv_server_mode;
    std_srvs::Empty srv_stop_task;
    std_srvs::Empty srv_show_maps;
    std_srvs::SetBool srv_toggle_explore;
    std_srvs::SetBool srv_return_to_dock;
    main_pkg::recieve_task_name srv_recieve_task_name;
    main_pkg::navMode srv_change_navMode;

    //A enum with 2 possibilities defined
    enum navMode
    {
        automatic,
        operation
    };
    navMode _navMode = operation;

public:
private:

    char sti[1023];
    std::vector<std::string> ops;
    //Function that removes redundant information from console
    void _menuLines(){std::cout << "------------------------------------" << std::endl;}
    //Menu that shows when trying to create a route
        enum server_state
    {
        inactivate,
        taskCoordinates,
        kitchenPos
        //0 = inactivate no points can be stored
        //1 = points are stored to the task array
        //2 = points are stored to the kitchen position
    };
    void _createTask()
    {
        system("clear");
        int selection = 0;
        std::string nameTask;
        std::cout << "-----------------------\nEnter name for task: ";
        std::cin.ignore();
        std::getline(std::cin, nameTask);
        //Function for sending name of task to server nameTask(nameTask);
        srv_add_task.request.name = nameTask;
        bool e = client_add_task.call(srv_add_task);
        std::cout << "bool return "<< e << std::endl;
        //Function for changing server mode to allow for inserting points.
        _server_mode(taskCoordinates);

        while (selection != 1)
        {
            system("clear");
            std::cout << " -----------------------\n1. Stop creating task and go back to menu" << std::endl;
            std::cout << "Enter: ";
            std::cin >> selection;
        }

        //Function for incremeting vector
        client_stop_task.call(srv_stop_task);
        //Changing server_mode to 0
        _server_mode(inactivate);
    };

    void _automaticMapping()
    {
        std::string c;
        system("clear");
        //service start automatic mapping
        srv_toggle_explore.request.data = true;
        client_toggle_explore.call(srv_toggle_explore);
        _menuLines();
        std::cout << "Automatic mapping started" << std::endl;
        _menuLines();
        std::cout << "Press any key to return: ";
        std::cin.ignore();
        std::getline(std::cin, c);
        srv_toggle_explore.request.data = false;
        client_toggle_explore.call(srv_toggle_explore);
        srv_return_to_dock.request.data = true;
        client_return_to_dock.call(srv_return_to_dock);
    }

    void _saveMap(){
        std::cout << "Enter name for new map: ";
        std::string s;
        std::cin >> s;
        s = "rosrun map_server map_saver -f "+s;
        system(s.c_str());
    }

    void _showMaps()
    {
        std::cout << "Displaying list of maps. Enter the map number to load." << std::endl;
        char st[] = {'/','h','o','m','e'};
	    ops.clear();
        traverse(st, false);
        printf("%s\n\n", sti);
        for(u_int i = 0; i < ops.size(); i++)
            std::cout << i+1 << " " << ops[i] << std::endl;

        u_int mapNumber;
        std::cin >> mapNumber;
        std::string s = "rosrun map_server map_server ";
        mapNumber--;
	    s += ops[mapNumber];
        system(s.c_str());
        std::cout << ops[mapNumber] << " loaded." << std::endl;
        
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
            std::cin >> srv_server_mode.request.mode;
            client_turtlebot_job.call(srv_server_mode);
        }
        else
        {
            std::cout << "[ERROR] - No tasks defined: Press any key to return: " << std::endl;
            std::cin.ignore();
            std::cin.get();
        }

        
    }

    void _kitchenPoint(){
        system("clear");
        _server_mode(kitchenPos);
        std::cout << "Insert kitchen point - press any key to return" << std::endl;
        std::cin.ignore();
        std::cin.get();

        _server_mode(inactivate);
    }

    void _change_navMode(){
	if(_navMode == automatic)
		_navMode = operation;
	else
		_navMode = automatic;


        srv_change_navMode.request.mode = _navMode;
        client_change_navMode.call(srv_change_navMode);

	std::cout <<"Changing navmode to: " << _navMode << std::endl;
    }

    /*void _chargingPoint(){ //Bliver ikke brugt længere
        system("clear");
        _server_mode(chargingPos);
        std::cout << "Insert charging point - press any key to return" << std::endl;
        std::cin.ignore();
        std::cin.get();

        _server_mode(inactivate);

    }*/

    void _server_mode(server_state s){

	srv_server_mode.request.mode = (int)s;

        client_server_mode.call(srv_server_mode);
        _menuLines();
        std::cout << "Server mode chaged " << s << std::endl;
        _menuLines();
    }
    void _menu()
    {
        int c = 1;

        while (c != 0)
        {
            system("clear");
            std::cout << "----------------------------" << std::endl;
            std::cout << "1. Create route for Turtlebot" << std::endl;
            std::cout << "2. Send task for Turtlebot to perform" << std::endl;
            std::cout << "3. Change operational mode. Current: "; 
	    std::string mode = "Manual"; if(_navMode==0) mode="Automatic"; std::cout << mode << std::endl;
            std::cout << "4. Start automatic mapping" << std::endl;
            std::cout << "5. Insert kitchen point" << std::endl;
            std::cout << "6. Save map" << std::endl;
            std::cout << "7. Load map" << std::endl;
            std::cout << "----------------------------" << std::endl;
	    std::cout << "B0: Start- or continue route \nB1: Move to kitchen \nB2: Move to dock" << std::endl;
	    std::cout << "\n----------------------------" << std::endl;
            std::cout << "Select option: ";
            std::cin >> c;
            while (1 > c > 7)
            {
                std::cin >> c;
            }

            switch (c){
                case 1:
                    _createTask();
                    break;
                case 2:
                    _sendTask();
                    break;
		case 3:
		    _change_navMode();
		    break;
                case 4:
                    _automaticMapping();
                    break;
                case 5:
                    _kitchenPoint();
                    break;
                case 6:
                    _saveMap();
                    break;
                case 7:
                    _showMaps();
                    break;

                default:
                    std::cout << "error" << std::endl;
                    break;
                }
            
            
            
        }
    }

    void traverse(char *fn, bool canAdd) 
    {
        DIR *dir;
        struct dirent *entry;
        char path[1023];
        struct stat info;
        if ((dir = opendir(fn)) != NULL){
            while ((entry = readdir(dir)) != NULL) {
                std::string s = entry->d_name;
                if (s.find(".pgm") != std::string::npos && !canAdd){
                    strncpy(sti,fn,1023);
                    traverse(fn, true);
                    return;
                }
                else if (entry->d_name[0] != '.') {
                    if(canAdd && s.find(".yaml") != std::string::npos)  
                        ops.push_back(s);
                    strcpy(path, fn);
                    strcat(path, "/");
                    strcat(path, entry->d_name);
                    stat(path, &info);
                    if (S_ISDIR(info.st_mode))  
                        traverse(path, false);
                }
            }
            closedir(dir);
        }
        
    }

public:
    Menu()
    {
        std::cout << 4 << std::endl;
        _menu();
        std::cout << 5 << std::endl;
    }
    ~Menu() {}
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "interface");
    Menu e;
    //menu();
    ros::spin();
}
