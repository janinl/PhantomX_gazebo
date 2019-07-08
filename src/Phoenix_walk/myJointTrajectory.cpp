#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "sensor_msgs/JointState.h"

using namespace std;

ros::Publisher jointGoals_pub;
control_msgs::FollowJointTrajectoryGoal trajectory;
int currentTrajectoryPoint = 0;
ros::Time last_publish_time;
vector<double> currentPositions(18);

void jointTrajectoryCommandCallback(const control_msgs::FollowJointTrajectoryGoal::ConstPtr &msg)
{
  // {joint_names: ["j_tibia_lf", "j_tibia_lm", "j_tibia_lr", "j_tibia_rf", "j_tibia_rm", "j_tibia_rr", "j_thigh_lf", "j_thigh_lm", "j_thigh_lr", "j_thigh_rf", "j_thigh_rm", "j_thigh_rr"], points: [ {positions:[0,0,0,0,0,0,0,0,0,0,0,0], time_from_start: [1,0]}, {positions:[-1,0,-1,0,1,0, 0,0,0,0,0,0], time_from_start: [2,0]}, {positions:[0,0,0,0,0,0,0,0,0,0,0,0], time_from_start: [3,0]}, {positions:[0,-1,0,1,0,1, 0,0,0,0,0,0], time_from_start: [4,0]}, {positions:[0,0,0,0,0,0,0,0,0,0,0,0], time_from_start: [5,0]}, ]}
  std::cout << "jointTrajectoryCommandCallback" << *msg << std::endl;
  trajectory = *msg;
  currentTrajectoryPoint = 0;
}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  //std::cout << "jointStateCallback" << std::endl;
  static int messageAlreadyPrinted1 = 0;
  if (trajectory.trajectory.points.empty())
  {
    if (messageAlreadyPrinted1++ == 0)
      std::cerr << "No trajectory pending" << std::endl;
    else
      std::cerr << ".";
    return;
  }
  messageAlreadyPrinted1 = 0;

  static int messageAlreadyPrinted2 = 0;
  ros::Time t1 = ros::Time::now();
  ros::Time t2 = last_publish_time + ros::Duration(2.0);
  if (t1 < t2)
  {
    if (messageAlreadyPrinted2++ == 0)
      std::cerr << "Too early. Ignoring joint state message" << std::endl;
    else
      std::cerr << ".";
    return;
  }
  messageAlreadyPrinted2 = 0;

  std::cout << "jointStateCallback msg: " << *msg << std::endl;
  std::cout << "currentTrajectoryPoint: " << currentTrajectoryPoint << std::endl;

  control_msgs::FollowJointTrajectoryGoal trajectoryWithOnePoint = trajectory;
  //trajectoryWithOnePoint.trajectory.joint_names = trajectory.trajectory.joint_names;
  trajectoryWithOnePoint.trajectory.points.clear();
  trajectoryWithOnePoint.trajectory.points.push_back(trajectory.trajectory.points[currentTrajectoryPoint]);
  // Calculate updated velopcity to reach each goal
  trajectoryWithOnePoint.trajectory.points[0].velocities.clear();
  for (int i=0; i<trajectoryWithOnePoint.trajectory.points[0].positions.size(); ++i) {
    double radians = abs(trajectoryWithOnePoint.trajectory.points[0].positions[i] - currentPositions[i]);
    double radiansPerSec = radians / 1.0; // todo
    trajectoryWithOnePoint.trajectory.points[0].velocities.push_back(radiansPerSec);
  }
  currentPositions = trajectoryWithOnePoint.trajectory.points[0].positions;

  jointGoals_pub.publish(trajectoryWithOnePoint);

  currentTrajectoryPoint++;
  currentTrajectoryPoint %= trajectory.trajectory.points.size();

  // Make sure we don't update too often
  last_publish_time = ros::Time::now();
}

vector<trajectory_msgs::JointTrajectoryPoint> points;

control_msgs::FollowJointTrajectoryGoal gaitTrajectory;

void initRosPublishers(ros::NodeHandle &n)
{
  jointGoals_pub = n.advertise<control_msgs::FollowJointTrajectoryGoal>("/webbie1/joint_goals", 1);

  std::cout << "Waiting for topic connections to be ready..." << std::endl;
  // Wait for all topic connections to be ready
  while (0 == jointGoals_pub.getNumSubscribers())
  {
    ROS_INFO("Waiting for subscribers to connect");
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "myJointTrajectory");
  ros::NodeHandle n;

  initRosPublishers(n);
  ros::Subscriber jointTrajectoryCommandSub(n.subscribe("/webbie1/joint_trajectory/command", 1, jointTrajectoryCommandCallback));
  ros::Subscriber jointStateSub(n.subscribe("/phantomx/joint_states", 1, jointStatesCallback));

  std::cout << "Starting" << std::endl;

  while (ros::ok())
  {

    ros::spin();
    return 0;
  }

  //	ros::spin();

  return 0;
}