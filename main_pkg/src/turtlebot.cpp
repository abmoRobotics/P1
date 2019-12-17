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
//#include <rate.h>

void debug(std::string a)
{
    enum mode
    {
        debug,
        operate,
    };
    mode mode = debug;

    if (mode == debug)
    {
        std::cout << "Test: " << a << std::endl;
    }
}

//Global variable as it is used in two classes
//Location in front of charging station
geometry_msgs::PointStamped chargingPoint;

/**
 * Class for moving the turtlebot is defined.
 *
 * This is a base class used by follwing classes: MoveBase, Reverse
 */
class moveCommands
{
    //Actions is defined.
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

protected:
    bool _move_base(move_base_msgs::MoveBaseGoal goal)
    {
        goal.target_pose.pose.orientation.w = 1.0;
        MoveBaseClient.waitForServer(); //Waiting until connection to action is established
        debug("9");
        MoveBaseClient.sendGoal(goal); //When connection is established send the goal to the action server
        debug("10");
        MoveBaseClient.waitForResult(); //Pause program until result is recieved
        debug("11");
        if (MoveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                debug("12");
                return true;
            }
            else
            {
                debug("13");
                return false;
            }
        
        ;
    }
    bool _cancenl_goals(){
        MoveBaseClient.cancelAllGoals();
    }
    moveCommands() : MoveBaseClient("move_base", true) {} //MoveBaseClient is initialized
};


class Reverse : public moveCommands
{
protected:
    enum charger_state
    {
        DISCHARGING = 0,
        DOCKING_CHARGED = 2,
        DOCKING_CHARGING = 6,
        ADAPTER_CHARGED = 18,
        ADAPTER_CHARGING = 22,
    };

    charger_state _chargingState;
    geometry_msgs::Twist t;
    ros::NodeHandle _nh;
    ros::Subscriber sub = _nh.subscribe("/mobile_base/events/power_system", 0, &Reverse::dockingPos, this);
    ros::Subscriber sub1 = _nh.subscribe("/mobile_base/sensors/core", 0, &Reverse::chargingState, this);
    ros::Subscriber subOdom = _nh.subscribe("/odom", 0, &Reverse::position, this);
    ros::Publisher cmd_vel = _nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
    geometry_msgs::PointStamped currentPos, dockPos;

    actionlib::SimpleActionServer<main_pkg::reverseAction> _as;
    std::string _actionName;
    main_pkg::reverseFeedback _feedback;
    main_pkg::reverseResult _result;
    bool docking;

public:
    Reverse(std::string name) : _as(_nh, name, boost::bind(&Reverse::executeCB, this, _1), false),
                                _actionName(name)
    {
        _as.start();
        std::cout << "started" << std::endl;
    }

    void executeCB()
    {
        if (_chargingState == DISCHARGING)
        {
            if (chargingPoint.point.x || chargingPoint.point.y || chargingPoint.point.z)
            {
                _result.result = 1;
            }
            else
            {
                _result.result = 0;
            }
            std::cout << "not in dock" << std::endl;
            return;
        }
        ros::Rate loop_rate(100);
        debug("5");
        bool success = true;
        t.linear.x = -0.2;
        double time = ros::Time::now().toSec();
        double distance = 0;
        std::cout << "Backing up" << std::endl;
        while (ros::ok() && distance < 0.4 && ros::Time::now().toSec() - time < 2.5)
        {
            cmd_vel.publish(t);
            std::cout << "Backing up" << std::endl;
            distance = sqrt(std::pow(dockPos.point.x - currentPos.point.x, 2) + std::pow(dockPos.point.y - currentPos.point.y, 2));
            std::cout << "distance: "<<distance << std::endl;
            
            if (!ros::ok())
            {
                success = false;
                break;
            }
            loop_rate.sleep();
        }

    std::cout << "1: before: "<<chargingPoint << std::endl;
     //chargingPoint = currentPos;
    std::cout << "1: after: "<<chargingPoint << std::endl;
        

       double angular_speed = 0.5;
       double relative_angle = 6.2831852/2;
   
       t.linear.x=0;
       t.linear.y=0;
       t.linear.z=0;
       t.angular.x = 0;
       t.angular.y = 0;
   
       t.angular.z = abs(angular_speed);
    
       double t0 = ros::Time::now().toSec();
       double current_angle = 0;
   
       while(current_angle < relative_angle){
           cmd_vel.publish(t);
           double t1 = ros::Time::now().toSec();
           current_angle = angular_speed*(t1-t0);
       }
        t.angular.z = 0;
        cmd_vel.publish(t);
        //Public cmd_vel
    }

    void executeCB(const main_pkg::reverseGoalConstPtr &goal)
    {
        if (_chargingState == DISCHARGING)
        {
            if ( chargingPoint.point.x || chargingPoint.point.y || chargingPoint.point.z)
            {
                _result.result = 1;
            }
            else
            {
                _result.result = 0;
            }
            _as.setSucceeded(_result);
            return;
        }
        ros::Rate loop_rate(100);
        debug("5");
        bool success = true;
        t.linear.x = -0.2;
        double distance = 0;
        std::cout << "Backing up" << std::endl;
        std::cout << goal->distance << std::endl;
        double time =ros::Time::now().toSec();
        while (ros::ok() && distance < 0.4 && ros::Time::now().toSec() - time < 2.5)
        {
            cmd_vel.publish(t);
            std::cout << "Backing up" << std::endl;
            _feedback.status = sqrt(std::pow(dockPos.point.x - currentPos.point.x, 2) + std::pow(dockPos.point.y - currentPos.point.y, 2));
            distance = _feedback.status;
            _as.publishFeedback(_feedback);
            if (_as.isPreemptRequested() || !ros::ok())
            {
                _as.setPreempted();
                success = false;
                break;
            }
            loop_rate.sleep();
        }

        
        
       double angular_speed = 0.5;
       double relative_angle = 3.14159265*3;
   
       t.linear.x=0;
       t.linear.y=0;
       t.linear.z=0;
       t.angular.x = 0;
       t.angular.y = 0;
   
       t.angular.z = angular_speed;
    
       double t0 = ros::Time::now().toSec();
       double current_angle = 0;
        std::cout << "hello " << std::endl;
        std::cout << "hello " << std::endl;
        std::cout << "hello " << std::endl;
        std::cout << "hello " << std::endl;
        std::cout << "hello " << std::endl;
       while(current_angle < relative_angle){
           cmd_vel.publish(t);
           double t1 = ros::Time::now().toSec();
           current_angle = angular_speed*(t1-t0);
        //std::cout << "angle: "<<current_angle << std::endl;
       }
        std::cout << "hello 2" << std::endl;
        std::cout << "hello 2" << std::endl;
        std::cout << "hello 2" << std::endl;
        std::cout << "hello 2" << std::endl;
        std::cout << "hello 2" << std::endl;
        std::cout << "hello 2" << std::endl;
        t.angular.z = 0;
        cmd_vel.publish(t);
        //Public cmd_vel
        if (success)
        {   

        std::cout << "hello 3" << std::endl;
        std::cout << "hello 3" << std::endl;
        std::cout << "hello 3" << std::endl;
        std::cout << "hello 3" << std::endl;
        std::cout << "hello 3" << std::endl;
            _result.result = _feedback.status;
        
    std::cout << "2: before: "<<chargingPoint << std::endl;
         chargingPoint = currentPos;
    std::cout << "2: after: "<<chargingPoint << std::endl;
            _as.setSucceeded(_result);
        }
    }

    void dockingPos(const kobuki_msgs::PowerSystemEvent::ConstPtr &state)
    {
        //todo add doskPos = NULLs
        ROS_INFO("Charge info received.");
        if (state->event == state->PLUGGED_TO_DOCKBASE || state->event == state->CHARGE_COMPLETED)
        {
            docking = true;
            dockPos = currentPos;
            std::cout << "Lade pois" << std::endl;
        }
        else if (state->event == state->PLUGGED_TO_ADAPTER || state->event == state->UNPLUGGED)
        {
            docking = false;
        }
        // ROS_INFO(std::to_string(state->event));
    }
    void position(const nav_msgs::Odometry::ConstPtr &msg)
    {
        //save position
        currentPos.point = msg->pose.pose.position;
        currentPos.header.frame_id = msg->header.frame_id;
        currentPos.header.stamp = ros::Time::now();
    }

    void chargingState(const kobuki_msgs::SensorState::ConstPtr &msg)
    {
        _chargingState = (charger_state)msg->charger;
    }

    bool returnToDock(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res)
    {
        debug("Return to dock");
        moveCommands::_cancenl_goals();
        if  (chargingPoint.point.x || chargingPoint.point.y || chargingPoint.point.z)
        {
            debug( "chargingPoint exists!");
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "odom";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position = chargingPoint.point;
            moveCommands::_move_base(goal);
        }
    }
};

class MoveBase : moveCommands
{
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
    ros::NodeHandle nh;
    //Actions are defined
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    //Services are defined
    ros::ServiceClient _client_receive_pose_array = nh.serviceClient<main_pkg::poseArray_srv>("get_job");                 //Service for recieving the next array of points(a task) from the server
    ros::ServiceClient _client_receive_pose_kitchen = nh.serviceClient<main_pkg::pointStamped_srv>("get_pose_kitchen");   //Service for getting the pose of the kitchen point
    ros::ServiceClient _client_receive_pose_charging = nh.serviceClient<main_pkg::pointStamped_srv>("get_pose_charging"); //Service for getting the pose of the charging point
    ros::ServiceClient _client_receive_navMode = nh.serviceClient<main_pkg::navMode>("get_navMode");
    ros::ServiceServer serv = nh.advertiseService("return_to_dock", &MoveBase::_moveToDock, this);

    //Subscribers
    ros::Subscriber sub = nh.subscribe("/mobile_base/events/button", 0, &MoveBase::_button_event, this);         //
    ros::Subscriber batterySub = nh.subscribe("/mobile_base/sensors/core", 1, &MoveBase::batteryCallback, this); //
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
    int minimum_battery_pct = 98;            //Battery % when it returns to dock
    int kobuki_max_charge = 163;             //Battery volt at full charge
    int current_battery = kobuki_max_charge; //Current battery in volt
    int current_dock_state = 2;
public:
    int job_size()
    {
        return _srv_receive_pose_array.response.arr.poses.size();
    }

    void _receive_pose_array()
    {
        //debug("1");
        _client_receive_pose_array.call(_srv_receive_pose_array);

        //debug("2");
        if (!_srv_receive_pose_array.response.arr.poses.empty())
        {
            length_job = _srv_receive_pose_array.response.arr.poses.size();
            _send_markers(_srv_receive_pose_array.response);
            debug("3");
            //_send_goal(_srv_receive_pose_array.response);
            debug("4");
        }
        else
        {
            _delete_markers();
            //std::cout << "No jobs pending" << std::endl;
        }
    }


    double B0 = 0;
    void _button_event(const kobuki_msgs::ButtonEvent::ConstPtr &msg)
    {
        if (msg->state == msg->PRESSED)
        {
            if (msg->button == msg->Button0)
            {
                B0 = ros::Time::now().toSec();
                std::cout << "Time is: " << B0 << std::endl;
            }
            else if (msg->button == msg->Button1)
            {
                _moveToKitchen();
            }

            else if (msg->button == msg->Button2)
            {
                _moveToDock();
            }
        }

        if (msg->state == msg->RELEASED)
        {
            if (msg->button == msg->Button0)
            {
                B0 = ros::Time::now().toSec() - B0;
                std::cout << "Time difference is: " << B0 << std::endl;
                if (B0 < 3)
                {
                    _send_task();
                }
                else if (B0 > 3)
                {
                    B0 = 0;
                    _srv_receive_pose_array.response.arr.poses.clear();
                    std::cout << "All points in route cleared" << std::endl;
                }
            }
        }
    }

    void _send_task()
    {
        _get_navMode(); //Get navmode from server
        if(current_dock_state != 0){
            Reverse rev("mover");
            rev.executeCB();
        }
        switch (_navMode)
        {
        case navMode::automatic:
            debug("Auto");
            for (int i = 0; i < length_job; i++)
            {
                _send_goal(_srv_receive_pose_array.response);
            }
            break;
        case navMode::operation:
            debug("operation");
            _send_goal(_srv_receive_pose_array.response);
            break;
        default:
            break;
        }
    }

    int _send_goal(main_pkg::poseArray_srv::Response p)
    {
        if (!p.arr.poses.empty())
        {
            move_base_msgs::MoveBaseGoal goal;
            debug("5");
            goal.target_pose.pose = p.arr.poses[0];
            debug("6");
            goal.target_pose.header.frame_id = p.arr.header.frame_id;
            debug("7");
            goal.target_pose.header.stamp = ros::Time::now();
            debug("8");
            if(moveCommands::_move_base(goal)){
                _srv_receive_pose_array.response.arr.poses.erase(_srv_receive_pose_array.response.arr.poses.begin());
            } else
            {
                debug("14ERROR");
            }
        }
    }

    int _send_goal(main_pkg::pointStamped_srv::Response p)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose.position = p.pose.point;
        goal.target_pose.pose.orientation.w = 1.0;
        goal.target_pose.header.frame_id = p.pose.header.frame_id;
        goal.target_pose.header.stamp = ros::Time::now();

        std::cout << "Target point:" << goal.target_pose.pose << std::endl;

        MoveBaseClient.waitForServer();
        MoveBaseClient.sendGoalAndWait(goal);
        if (MoveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            std::cout << "SUCCES" << std::endl;
        }
        else
        {
            std::cout << "ERROR" << std::endl;
        }
    }

    void _delete_markers()
    {
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_array;
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(marker);
        pub_marker.publish(marker_array);
    }

    void _send_markers(main_pkg::poseArray_srv::Response pose_array)
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.ns = "bus_stops";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0.7071;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 0.7071;

        marker.lifetime = ros::Duration();

        visualization_msgs::MarkerArray marker_array;
        for (int i = 0; i < pose_array.arr.poses.size(); i++)
        {
            marker.header.frame_id = pose_array.arr.header.frame_id;
            marker.id = i;
            marker.pose.position = pose_array.arr.poses[i].position;
            marker.pose.position.z += marker.scale.x;
            marker_array.markers.push_back(marker);
        }

        pub_marker.publish(marker_array);
        marker_array.markers.clear();
    }
    //Battery callback
    void batteryCallback(const kobuki_msgs::SensorState::ConstPtr &msg)
    {
        //Store variables for batterycheck()
        current_battery = msg->battery;
        current_dock_state = msg->charger;
    }

    bool battery_check() //Returns true if it needs to recharge
    {
        float batterypct = float(current_battery) / float(kobuki_max_charge) * 100; //Calculate pct
        //ROS_INFO("pct: %f", batterypct);

        if (current_dock_state == 0 && batterypct < minimum_battery_pct)
        { //Not in dock and under minimal%
            debug("Battery is under minimal charge");

            _moveToDock();

            return true;
        }
        //In dock and under minimal%
        else if (current_dock_state == 6 && batterypct < minimum_battery_pct)
        {
            return true;
        }
        return false;
    }

    void _moveToDock()
    {

        moveCommands::_cancenl_goals();
        std::cout << "Moving to the dock. Point is: " << chargingPoint << std::endl;

        if (chargingPoint.point.x || chargingPoint.point.y || chargingPoint.point.z)
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "odom";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position = chargingPoint.point;
            ROS_INFO("Charging point found!");
            moveCommands::_move_base(goal);
            system("roslaunch kobuki_auto_docking activate.launch --screen");
        }
        else
            ROS_INFO("There is no point set for charging");
    }
    bool _moveToDock(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res)
    {
        _moveToDock();
    }

    void _moveToKitchen()
    {
        debug("Moving to the kitchen");
        //Get point
        _client_receive_pose_kitchen.call(_srv_receive_pose_kitchen);

        main_pkg::pointStamped_srv::Response kitchenPoint = _srv_receive_pose_kitchen.response;

        //Check if pose_kitchen has been set (if not origo)
        if (kitchenPoint.pose.point.x != 0 || kitchenPoint.pose.point.y != 0 || kitchenPoint.pose.point.z != 0)
        {
            ROS_INFO("Kitchen point found!");
            _send_goal(kitchenPoint);
        }
        else
        {
            ROS_INFO("Kitchen point not found");
        }
    }

    //Get navmode from server
    void _get_navMode()
    {
        _client_receive_navMode.call(_srv_receive_navMode);

        _navMode = static_cast<navMode>(_srv_receive_navMode.response.mode);

        std::cout << "Navmode is: " << _navMode << std::endl;
    }

public:
    MoveBase() : MoveBaseClient("move_base", true)
    {
    }
};

//tydebugdef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
int main(int argc, char *argv[])
{
    std::cout << "Starter turtlebot" << std::endl;
    ros::init(argc, argv, "caterbot");
    char c;
    MoveBase e;
    Reverse r("mover");

    ros::Rate loop_rate(1);
    while (ros::ok)
    {
        if (e.job_size() == 0) //If robot doesn't have any jobs to perform
        {
            if (!e.battery_check())
            { //If it doesn't require recharging
                e._receive_pose_array();
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}
