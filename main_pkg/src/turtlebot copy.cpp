#include <ros/ros.h>
#include <main_pkg/poseArray_srv.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamdebugd.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


class Movement{
    public:
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<main_pkg::poseArray_srv>("get_job");

    
    geometry_msgs::PoseArray goal;

    bool _request_job(){
            main_pkg::poseArray_srv srv;
            client.call(srv);
            goal = srv.response.arr;
            for(int i=0; i<srv.response.arr.poses.size(); i++){
                std::cout << srv.response.arr.poses[i].position << std::endl;
        }
    }  
};

void debug(std::string a){
    enum mode {
        debug, operate,
    }
    mode mode = operate;
    if (mode==debug){
        std::cout << "Test: " << a << std::endl;
    }
    
}

class MoveBase{
    private:
    //nodeHandle defined
    ros::NodeHandle nh;
    //Actions are defined
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    //Services are defined
    ros::ServiceClient _client_recieve_pose_array = nh.serviceClient<main_pkg::poseArray_srv>("get_job");
    //Service message for current job
    main_pkg::poseArray_srv _srv_recieve_pose_array;

    //Publisher for markers
    ros::Publisher pub_marker = nh.advertise<visualization_msgs::MarkerArray>(
            "route_markers", 1);


    public:
    void _recieve_pose_array(){
        debug("1");
        _client_recieve_pose_array.call(_srv_recieve_pose_array);
        debug("2");
        if(!_srv_recieve_pose_array.response.arr.poses.empty()){
            _send_markers(_srv_recieve_pose_array.response);
            debug("3");
            _send_goal(_srv_recieve_pose_array.response);
            debug("4");  
            } else {
            std::cout << "No jobs debugnding" << std::endl;
            }
            _srv_recieve_pose_array.response.arr.poses.clear();
        } 
    

    void _send_goal(main_pkg::poseArray_srv::Response pose_array){
        move_base_msgs::MoveBaseGoal goal;
        for(int i = 0; i < pose_array.arr.poses.size(); i++){
            debug("5");
            goal.target_pose.pose = pose_array.arr.poses[i];
            debug("6");
            goal.target_pose.header.frame_id = pose_array.arr.header.frame_id;
            debug("7");
            goal.target_pose.header.stamp = ros::Time::now();
            debug("8");
            MoveBaseClient.waitForServer();
            debug("9");
            MoveBaseClient.sendGoal(goal);
            debug("10");
            MoveBaseClient.waitForResult();
            debug("11");
            if (MoveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                std::cout << "Reached goal: " << i+1 << " OF " << pose_array.arr.poses.size() << std::endl;
            }
             else{
                 std::cout << "ERROR - Current State: " << MoveBaseClient.getState().toString() << std::endl;
             }

             
        }
        _delete_markers();
        

    }

    void _delete_markers(){
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
        marker.tydebug = visualization_msgs::Marker::ARROW;
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
        for(int i = 0; i < pose_array.arr.poses.size(); i++){ 
            marker.header.frame_id = pose_array.arr.header.frame_id;
            marker.id = i;
            marker.pose.position = pose_array.arr.poses[i].position;
            marker.pose.position.z += marker.scale.x;
            marker_array.markers.push_back(marker);
        }
        
        pub_marker.publish(marker_array);
        marker_array.markers.clear();                                            
    }

    public:
    MoveBase() : MoveBaseClient("move_base", true){

    }
};



//tydebugdef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "caterbot");
    MoveBase e;
    ros::Rate loop_rate(1);
    while(ros::ok){

        e._recieve_pose_array();

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}
