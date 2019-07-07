#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

using namespace std;

vector<ros::Publisher> joint_pubs;
vector<string> jointNames = {
    "j_c1_lf", "j_c1_lm", "j_c1_lr", "j_c1_rf", "j_c1_rm", "j_c1_rr",
    "j_thigh_lf", "j_thigh_lm", "j_thigh_lr", "j_thigh_rf", "j_thigh_rm", "j_thigh_rr",
    "j_tibia_lf", "j_tibia_lm", "j_tibia_lr", "j_tibia_rf", "j_tibia_rm", "j_tibia_rr"};

void jointGoalsCallback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr &msg)
{
  std::cout << "jointGoalsCallback" << *msg << std::endl;
  assert(jointNames.size() == msg->trajectory.joint_names.size());
  assert(msg->trajectory.points.size() == 1);
  for (int i = 0; i < jointNames.size(); ++i)
  {
    assert(jointNames[i] == msg->trajectory.joint_names[i]);
    std_msgs::Float64 msg2;
    msg2.data = msg->trajectory.points[0].positions[i];
    joint_pubs[i].publish(msg2);
    std::cout << "Sending " << msg2 << " to " << jointNames[i] << std::endl;
  }
}

void initRosPublishers(ros::NodeHandle &n)
{

  for (auto &jointName : jointNames)
  {
    string topicName = "/phantomx/" + jointName + "_position_controller/command";
    joint_pubs.push_back(n.advertise<std_msgs::Float64>(topicName, 1));
  }
  /*
   std::cout << "Waiting for topic connections to be ready..." << std::endl;
  // Wait for all topic connections to be ready
  while (0 == jointGoals_pub.getNumSubscribers())
  {
    ROS_INFO("Waiting for subscribers to connect");
    ros::Duration(1.0).sleep();
  }
*/
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jointGoalsSplit");
  ros::NodeHandle n;

  initRosPublishers(n);
  ros::Subscriber jointGoalsSub(n.subscribe("/webbie1/joint_goals", 1, jointGoalsCallback));

  std::cout << "Starting" << std::endl;
  ros::spin();
  return 0;
}