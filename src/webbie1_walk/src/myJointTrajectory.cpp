#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

using namespace std;

ros::Publisher jointGoals_pub;
ros::Publisher gazeboTrajectoryControllerForSingleStep_pub;
control_msgs::FollowJointTrajectoryGoal trajectory;
int currentTrajectoryPoint = 0;
ros::Time last_publish_time;
vector<double> currentPositions(18);
bool noTrajectoryPointWasEverSent = true;

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

#ifdef ONE_STEP_EVERY_2_SECONDS
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

#else

  // Have we reached the trajectory point goal?
  std::cout << "jointStateCallback msg: " << *msg << std::endl;
  std::cout << "currentTrajectoryPoint: " << currentTrajectoryPoint << std::endl;

  bool closeEnough = true;
  for (int i=0; i<trajectory.trajectory.points[currentTrajectoryPoint].positions.size(); ++i) {
    currentPositions[i] = msg->position[i];
    double distanceToGoal = abs(trajectory.trajectory.points[currentTrajectoryPoint].positions[i] - currentPositions[i]);
    cout << "Distance from goal: i=" << i << ", distanceToGoal=" << distanceToGoal
    << ", currentPositions=" << currentPositions[i] << ", goal=" << trajectory.trajectory.points[currentTrajectoryPoint].positions[i] << endl;
    if (distanceToGoal > 0.1)
      closeEnough = false;
  }

  if (noTrajectoryPointWasEverSent)
  {
    noTrajectoryPointWasEverSent = false;
    cout << "noTrajectoryPointWasEverSent => Sending first point" << endl;
  }
  else {
    if (closeEnough)
      cout << "Close enough! Moving to the next trajectory point." << endl;
    else {
      cout << "Too far from goal. Keeping current goal point active." << endl;
      return;
    }
  }

#endif

  int previousTrajectoryPoint = currentTrajectoryPoint;
  currentTrajectoryPoint++;
  currentTrajectoryPoint %= trajectory.trajectory.points.size();
  cout << "Sending next trajectory point: " << currentTrajectoryPoint << endl;


  // Calculate delay between the 2 points
  ros::Duration durationBetweenPoints;
  if (currentTrajectoryPoint==0)
    durationBetweenPoints = trajectory.trajectory.points[currentTrajectoryPoint].time_from_start;
  else
    durationBetweenPoints = trajectory.trajectory.points[currentTrajectoryPoint].time_from_start - trajectory.trajectory.points[previousTrajectoryPoint].time_from_start;
  cout << "durationBetweenPoints:" << durationBetweenPoints << endl;
  double secondsBetweenPoints = durationBetweenPoints.toSec();
  cout << "secondsBetweenPoints:" << secondsBetweenPoints << endl;


  // Preparing topicmessage
  trajectory_msgs::JointTrajectory trajectoryWithOnePoint = trajectory.trajectory;
  //trajectoryWithOnePoint.trajectory.joint_names = trajectory.trajectory.joint_names;
  trajectoryWithOnePoint.points.clear();
  trajectoryWithOnePoint.points.push_back(trajectory.trajectory.points[currentTrajectoryPoint]);
  // Calculate updated velocity to reach each goal
  trajectoryWithOnePoint.points[0].velocities.clear();
  for (int i=0; i<trajectoryWithOnePoint.points[0].positions.size(); ++i) {
    double radians = abs(trajectoryWithOnePoint.points[0].positions[i] - currentPositions[i]);
    double radiansPerSec = radians / secondsBetweenPoints;
    trajectoryWithOnePoint.points[0].velocities.push_back(radiansPerSec);
  }

  static ros::Duration last_time_from_start(0);
  //last_time_from_start += durationBetweenPoints;
  //trajectoryWithOnePoint.points[0].time_from_start = last_time_from_start;
  {
    static ros::Time overallStartingTime = ros::Time::now();
    trajectoryWithOnePoint.points[0].time_from_start = ros::Time::now() - overallStartingTime + durationBetweenPoints;
  }
  //currentPositions = trajectoryWithOnePoint.points[0].positions;

  cout << "Sending trajectoryWithOnePoint: " << trajectoryWithOnePoint << endl;
  jointGoals_pub.publish(trajectoryWithOnePoint);
  gazeboTrajectoryControllerForSingleStep_pub.publish(trajectoryWithOnePoint);


  // Make sure we don't update too often
  last_publish_time = ros::Time::now();
}

vector<trajectory_msgs::JointTrajectoryPoint> points;

void initRosPublishers(ros::NodeHandle &n)
{
  jointGoals_pub = n.advertise<trajectory_msgs::JointTrajectory>("/webbie1/joint_goals", 1);
  gazeboTrajectoryControllerForSingleStep_pub = n.advertise<trajectory_msgs::JointTrajectory>("/phantomx/trajectory_controller_for_single_step/command", 1);

  std::cout << "Waiting for topic connections to be ready..." << std::endl;
  // Wait for all topic connections to be ready
  while (0 == jointGoals_pub.getNumSubscribers()
      && 0 == gazeboTrajectoryControllerForSingleStep_pub.getNumSubscribers())
  {
    ROS_INFO("Waiting for subscribers to connect");
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "webbie1_myJointTrajectory");
  ros::NodeHandle n;

  initRosPublishers(n);
  ros::Subscriber jointTrajectoryCommandSub(n.subscribe("/webbie1/joint_trajectory/command", 1, jointTrajectoryCommandCallback));
  ros::Subscriber jointStateSub(n.subscribe("/webbie1/joint_states", 1, jointStatesCallback));

  std::cout << "Starting" << std::endl;

  while (ros::ok())
  {

    ros::spin();
    return 0;
  }

  //	ros::spin();

  return 0;
}