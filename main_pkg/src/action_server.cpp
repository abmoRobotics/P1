#include <ros/ros.h>

#include <move_base_msgs/MoveBaseGoal.h>// Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <main_pkg/testActionGoal.h>
#include <main_pkg/testAction.h>
#include <main_pkg/testGoal.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base");
  Client client("move_base", true); // true -> don't need ros::spin()
  client.waitForServer();
  move_base_msgs::MoveBaseGoal goal;
  // Fill in goal here
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");
  printf("Current State: %s\n", client.getState().toString().c_str());
  
  return 0;
}
