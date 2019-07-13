#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/time.h"

ros::Publisher head_pub;

int setValeurPoint(trajectory_msgs::JointTrajectory *trajectoire, int pos_tab, int val);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  head_pub = n.advertise<trajectory_msgs::JointTrajectory>("/phantomx/trajectory_controller_for_single_step/command", 0);

  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectoryPoint points_n;

  traj.header.frame_id = "base_link";

  traj.joint_names.resize(12);
  traj.joint_names = {"j_tibia_lf", "j_tibia_lm", "j_tibia_lr", "j_tibia_rf", "j_tibia_rm", "j_tibia_rr", "j_thigh_lf", "j_thigh_lm", "j_thigh_lr", "j_thigh_rf", "j_thigh_rm", "j_thigh_rr"};
  //"j_tibia_lf", "j_tibia_lm", "j_tibia_lr", "j_tibia_rf", "j_tibia_rm", "j_tibia_rr", "j_thigh_lf", "j_thigh_lm", "j_thigh_lr", "j_thigh_rf", "j_thigh_rm", "j_thigh_rr"
  /*
  traj.joint_names[0] = "j_tibia_lf";
  traj.joint_names[1] = "j_tibia_rf";
  traj.joint_names[2] = "j_thigh_lf";
  traj.joint_names[3] = "j_thigh_rf";
*/

  traj.points.resize(2);

  // Wait for all topic connections to be ready
  while (0 == head_pub.getNumSubscribers())
  {
    ROS_INFO("Waiting for subscribers to connect");
    ros::Duration(0.1).sleep();
  }

  int i(100);

  while (ros::ok())
  {
    traj.header.stamp = ros::Time::now();

    for (int k = 0; k < 2; ++k)
    {
      for (int j = 0; j < 12; j++)

      {
        //      setValeurPoint(&traj, j, i);
        traj.points[k].positions.resize(12);
        traj.points[k].positions[j] = (k + 1) % 2;
      }

      traj.points[k].time_from_start = ros::Duration(5 * (k + 1));
    }

    std::cout << "publishing trajectory. Num_subscribers=" << head_pub.getNumSubscribers() << std::endl;
    head_pub.publish(traj);
    std::cout << "publishing trajectory done" << std::endl;
    //ros::spinOnce();

    std::cout << "sleeping" << std::endl;
    ros::Duration(10).sleep();

    i++;
  }

  return 0;
}
