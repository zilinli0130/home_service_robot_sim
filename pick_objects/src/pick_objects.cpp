#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 4.0;
  goal.target_pose.pose.position.y = -1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal to pickup zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its pickup zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The robot reached the pickup zone successfully");
      ros::Duration(5.0).sleep();
      //Go to dropoff zone if the robot reached the pickup zone
      goal.target_pose.pose.position.x = -3.5;
      goal.target_pose.pose.position.y = -2.0;
      goal.target_pose.pose.orientation.w = 1.0;
  } else {
     ROS_INFO("The robot failed to reach the pickup zone");
  }

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal to dropoff zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("The robot reached the dropoff zone successfully");
      ros::Duration(5.0).sleep();
  } else {
     ROS_INFO("The robot failed to reach the droneoff zone");
  }

  return 0;
}
