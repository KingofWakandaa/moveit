// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include "Arm_class.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takethings");

//  if ( argc < 7 )
//  {
//    ROS_INFO(" ");
//    ROS_INFO("\tUsage:");
//    ROS_INFO(" ");
  //   ROS_INFO("\trosrun tiago_moveit_tutorial plan_arm_torso_ik  x y z  r p y");
  //   ROS_INFO(" ");
  //   ROS_INFO("\twhere the list of arguments specify the target pose of /arm_tool_link expressed in /base_footprint");
  //   ROS_INFO(" ");
  //   return EXIT_FAILURE;
  // }
  //
  // geometry_msgs::PoseStamped goal_pose;
  // goal_pose.header.frame_id = "base_footprint";
  // goal_pose.pose.position.x = atof(argv[1]);
  // goal_pose.pose.position.y = atof(argv[2]);
  // goal_pose.pose.position.z = atof(argv[3]);
  // goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[4]), atof(argv[5]), atof(argv[6]));

  Arm_class Arm_tiago;
  ros::NodeHandle nh;
  ros::subscriber sub_goal_pos = nh.subscribe("position",100,&Arm_class::getPose,&Arm_tiago)


  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_footprint";
  goal_pose.pose.position = Arm_tiago.position;
  goal_pose.pose.orientation = Arm_tiago.orientation;

  std::vector<std::string> torso_arm_joint_names;
  //select group of joints
  moveit::planning_interface::MoveGroup group_arm_torso("arm_torso");//???????
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(2.0);//speed up???


  moveit::planning_interface::MoveGroup::Plan my_plan;
  //set maximum time to find a plan
  group_arm_torso.setPlanningTime(5.0);
  bool success = group_arm_torso.plan(my_plan);

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();

  group_arm_torso.move();

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  // spinner.stop();
  ros::spin();
  return EXIT_SUCCESS;
}
