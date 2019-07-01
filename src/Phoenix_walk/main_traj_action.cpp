#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *acGripper;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_gazebo_message_wrapper");

  acGripper = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("phantomx/head_controller/follow_joint_trajectory", true);

  while (ros::ok())
  {
    control_msgs::FollowJointTrajectoryGoal msg;

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(0);
    point.positions.push_back(0);
    point.velocities.push_back(0);
    point.velocities.push_back(0);
    point.accelerations.push_back(0);
    point.accelerations.push_back(0);
    point.time_from_start = ros::Duration(0.1);
    msg.trajectory.points.push_back(point);

    msg.trajectory.joint_names.push_back("j_tibia_lf");
    msg.trajectory.joint_names.push_back("j_tibia_rf");

    // fill message header and sent it out
    msg.trajectory.header.frame_id = "gripper_palm_link";
    msg.trajectory.header.stamp = ros::Time::now();
    acGripper->sendGoal(msg);

    ros::spinOnce();
    std::cout << "sleeping" << std::endl;
    ros::Duration(1).sleep();
  }

  //	ros::spin();

  return 0;
}