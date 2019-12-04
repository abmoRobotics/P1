#include <iostream>


void ClearScreen();
void createMenu();
void sendRouteMenu();
void automaticMapping();
void createMenu_stop(std::string);
/*
void menu(){
    int c;
    std::cout << "1. create a route" << std::endl;
    std::cout << "2. Initiate automatic mapping" << std::endl;
    std::cout << "3. Choose task for robot" << std::endl;
    std::cin >> c;

    if (c==1) {
        std::cout << "Give the task a name" << std::endl;

    }
    else if (c==2){

    }
    else if (c==3){

    }
}*/


    void menu(){
        ClearScreen();
        int selection = 0;
        std::cout << "----------------------------" << std::endl;
        std::cout << "1. Create route for Turtlebot" << std::endl;
        std::cout << "2. Send task for Turtlebot to perform" << std::endl;
        std::cout << "3. Start automatic mapping" << std::endl;
        std::cout << "Select option: ";
        std::cin >> selection;

        switch(selection) {
            case 1:
                createMenu();
                break;
            case 2:
                sendRouteMenu();
                break;
            case 3:
                automaticMapping();
                break;
            default:
                std::cout << "faul";
                menu();
                break;
        }
    }

        void createMenu(){
            ClearScreen();
            int selection;
            std::string nameTask;
            std::cout <<"-----------------------" << std::endl;
            std::cout << "Enter name for task: ";
            std::cin >> nameTask;
            //Function for sending name to server nameTask(nameTask);
            createMenu_stop(nameTask);
            };

        void createMenu_stop(std::string a){
            ClearScreen();
            std::cout << "LOL " << a << std::endl;
            int selectionn = 0;
            std::cout << " -----------------------" << std::endl;
            std::cout << "1. Stop creating task and go back to menu" << std::endl;
            std::cout << "Enter: ";
            std::cin >> selectionn;
            switch(selectionn){
                case 1:
                    //function for incrementing
                    menu();
                default:
                    createMenu_stop(a);
            }
        }

        void sendRouteMenu(){
            ClearScreen();
            std::cout << "Send route" << std::endl;
        }
        void automaticMapping(){
            ClearScreen();
            std::cout << "Automatic mapping" << std::endl;
        }
          void ClearScreen()
            {
            std::cout << std::string( 100, '\n' );
            }



int main(int argc, char *argv[]){
    menu();
}