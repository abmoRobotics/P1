#ifndef TURTLEBOT
#define TURTLEBOT

#include <ros/ros.h>
#include <main_pkg/poseArray_srv.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/PowerSystemEvent.h>
#include <main_pkg/pointStamped_srv.h>
#include <main_pkg/reverseAction.h>
#include <main_pkg/navMode.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>

class TurtleCore
{
    private:
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    protected:
        TurtleCore();

        ros::NodeHandle nh;

        enum charger_state
        {
            DISCHARGING = 0,
            DOCKING_CHARGED = 2,
            DOCKING_CHARGING = 6,
            ADAPTER_CHARGED = 18,
            ADAPTER_CHARGING = 22,
        };

        int minimum_battery_pct = 98;            //Battery % when it returns to dock
        int kobuki_max_charge = 163;             //Battery volt at full charge
        int current_battery = kobuki_max_charge; //Current battery in volt
        
        charger_state _chargingState;
        geometry_msgs::Twist t;

        //Subscribers
        ros::Subscriber sub  =   nh.subscribe("/mobile_base/events/power_system", 0, &Reverse::dockingPos, this);
        ros::Subscriber sub2 =   nh.subscribe("/mobile_base/events/button", 0, &MoveBase::_button_event, this);         //
        ros::Subscriber sub3 =   nh.subscribe("/mobile_base/sensors/core", 1, &MoveBase::batteryCallback, this); //
        ros::Subscriber sub4 =   nh.subscribe("/odom", 0, &Reverse::position, this);

        ros::ServiceServer serv =    nh.advertiseService("return_to_dock", &MoveBase::_moveToDock, this);


        void batteryCallback(const kobuki_msgs::SensorState::ConstPtr &msg);
        bool _move_base(move_base_msgs::MoveBaseGoal goal);
        bool returnToDock();
        bool returnToDock(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res);


};
class Reverse : public TurtleCore
{
    protected:
        ros::Publisher cmd_vel = _nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
        geometry_msgs::PointStamped currentPos, dockPos, gatePos;
        
        actionlib::SimpleActionServer<main_pkg::reverseAction> _as;

        std::string _actionName;
        main_pkg::reverseFeedback _feedback;
        main_pkg::reverseResult _result;
        bool docking;
    public:
        Reverse(std::string name);
        void executeCB(const main_pkg::reverseGoalConstPtr &goal);
        void executeCB();
        void dockingPos(const kobuki_msgs::PowerSystemEvent::ConstPtr &state);
        void position(const nav_msgs::Odometry::ConstPtr &msg);
        void chargingState(const kobuki_msgs::SensorState::ConstPtr &msg);
        /* bool returnToDock(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res); */
};

class MoveBase : public TurtleCore
{
    public:
        MoveBase();
        int  job_size();
        void _receive_pose_array();
        void _button_event(const kobuki_msgs::ButtonEvent::ConstPtr &msg);
        void _send_task();
        int  _send_goal(main_pkg::poseArray_srv::Response p);
        int  _send_goal(main_pkg::pointStamped_srv::Response p);
        void _delete_markers();
        void _send_markers(main_pkg::poseArray_srv::Response pose_array);
        bool battery_check(); //Returns true if it needs to recharge
        /* void _moveToDock();
        bool _moveToDock(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res); */
        void _moveToKitchen();
        void _get_navMode();
    private:
        //A enum with 2 possibilities defined
        enum navMode
        {
            automatic,
            operation
        };
        //Variable for determining whether button should be pressed between navigation points
        navMode _navMode = operation;
        //NodeHandle defined
        //Services are defined
        ros::ServiceClient _client_receive_pose_array = nh.serviceClient<main_pkg::poseArray_srv>("get_job");                 //Service for recieving the next array of points(a task) from the server
        ros::ServiceClient _client_receive_pose_kitchen = nh.serviceClient<main_pkg::pointStamped_srv>("get_pose_kitchen");   //Service for getting the pose of the kitchen point
        ros::ServiceClient _client_receive_pose_charging = nh.serviceClient<main_pkg::pointStamped_srv>("get_pose_charging"); //Service for getting the pose of the charging point
        ros::ServiceClient _client_receive_navMode = nh.serviceClient<main_pkg::navMode>("get_navMode");

        //Custom service messages defined
        main_pkg::poseArray_srv _srv_receive_pose_array;
        main_pkg::pointStamped_srv _srv_receive_pose_kitchen;
        main_pkg::pointStamped_srv _srv_receive_pose_charging;
        main_pkg::navMode _srv_receive_navMode;

        //Publisher for markers
        ros::Publisher pub_marker = nh.advertise<visualization_msgs::MarkerArray>(
            "route_markers", 1);
        //Variables
        int length_job = 0;
        int current_dock_state = 2;
        double B0 = 0;
        //functions
        

};

#endif