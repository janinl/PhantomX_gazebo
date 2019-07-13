#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "sensor_msgs/JointState.h"

#include "mytypes.h"
#include "ax12Serial.hh"
#define DEFINE_HEX_GLOBALS
#define HEXMODE // default to hex mode
#include "Hex_Cfg.h"
#undef SOUND_PIN
#include "_Phoenix.h"
#include "Input_Controller_raspi.h"
#include "_Phoenix_Driver_AX12.h"
#include "_Phoenix_Code.h"

long unsigned int millis()
{
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long unsigned int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  return ms;
}

using namespace std;

void submitTrajectory(ros::Publisher &jointTrajectoryCommand_pub);


void jointStateCallback2(const sensor_msgs::JointState::ConstPtr &msg)
{
  //return;
  //std::cout << "jointStateCallback" << *msg << std::endl;
  for (int i = 12; i < msg->effort.size(); ++i)
  {
    if (std::abs(msg->effort[i]) > 2.79)
    {
      std::cout << "Large tibia effort detected: " << i << "\n"
                << *msg << std::endl;
      //webbie1Trajectory->cancelAllGoals();
      break;
    }
  }
}

vector<trajectory_msgs::JointTrajectoryPoint> points;
void saveTrajectoryPoint()
{
  trajectory_msgs::JointTrajectoryPoint point1;
  point1.positions = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  for (int LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++)
  {
    cout << "Leg " << LegIndex << endl;
    // CoxaAngle1 is between -1500 and 1500
    double coxa = cCoxaInv[LegIndex] ? -CoxaAngle1[LegIndex] : CoxaAngle1[LegIndex];
    double femur = cFemurInv[LegIndex] ? -FemurAngle1[LegIndex] : FemurAngle1[LegIndex];
    double tibia = cTibiaInv[LegIndex] ? -TibiaAngle1[LegIndex] : TibiaAngle1[LegIndex];
    coxa = coxa / 1800 * 3.14;
    femur = femur / 1800 * 3.14;
    tibia = tibia / 1800 * 3.14;
    cout << " CoxaAngle1: " << coxa << endl;
    cout << " FemurAngle1: " << femur << endl;
    cout << " TibiaAngle1: " << tibia << endl;
    point1.positions[5 - LegIndex] = coxa;
    point1.positions[11 - LegIndex] = femur;
    point1.positions[17 - LegIndex] = tibia;
  }
  points.push_back(point1);
}

void calculateTrajectoryPoints()
{
  setup();
  fWalking = true;
  g_InControlState.TravelLength.z = -100; // max -127
  g_InControlState.BodyPos.y = 80;
  GaitPosZ[1] = GaitPosZ[3] = GaitPosZ[5] = (g_InControlState.TravelLength.z/(short)g_InControlState.gaitCur.TLDivFactor);

  loop(true);
  saveTrajectoryPoint();
  loop(true);
  saveTrajectoryPoint();
  loop(true);
  saveTrajectoryPoint();
  loop(true);
  saveTrajectoryPoint();
  loop(true);
  saveTrajectoryPoint();
  loop(true);
  saveTrajectoryPoint();
  loop(true);
  saveTrajectoryPoint();
  loop(true);
  saveTrajectoryPoint();

  return;
  // For debugging: make half gait go back and forth
  //int pointnums[8] = {0, 1, 2, 1, 0, 7, 6, 7};
  points[3] = points[1];
  points[4] = points[0];
  points[5] = points[7];
  return;
  // For debugging: 1,2
  points[0] = points[1];
  points[1] = points[2];
  points.resize(2);
}

control_msgs::FollowJointTrajectoryGoal gaitTrajectory;
void calculateTrajectory()
{
  calculateTrajectoryPoints();

  /*
    trajectory_msgs::JointTrajectoryPoint point = points[0];
    //point.positions = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    point.velocities = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    point.accelerations = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    point.time_from_start = ros::Duration(2.0);
    gaitTrajectory.trajectory.points.push_back(point);

    //point.positions = {0, 0, 0, 0, 0, 0, 1, 1, 1, -1, -1, -1, 0, 0, 0, 0, 0, 0};
    //point.time_from_start += ros::Duration(2.0);
    //gaitTrajectory.trajectory.points.push_back(point);

    for (int i = 6; i < 12; ++i)
    {
      point.positions[i - 1] = 0;
      point.positions[i] = i < 9 ? -1 : 1;
      point.time_from_start += ros::Duration(1.0);
      //gaitTrajectory.trajectory.points.push_back(point);
    }
    */
  auto time_from_start = ros::Duration(0);
  for (int repeat = 0; repeat < 1; ++repeat)
    for (auto point : points)
    {
      //point.velocities = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      //point.accelerations = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      time_from_start += ros::Duration(1.0);
      point.time_from_start = time_from_start;
      gaitTrajectory.trajectory.points.push_back(point);
    }

  gaitTrajectory.trajectory.joint_names = {
      "j_c1_lf", "j_c1_lm", "j_c1_lr", "j_c1_rf", "j_c1_rm", "j_c1_rr",
      "j_thigh_lf", "j_thigh_lm", "j_thigh_lr", "j_thigh_rf", "j_thigh_rm", "j_thigh_rr",
      "j_tibia_lf", "j_tibia_lm", "j_tibia_lr", "j_tibia_rf", "j_tibia_rm", "j_tibia_rr"};

  // fill message header and sent it out
  gaitTrajectory.trajectory.header.frame_id = "whatever";
}

void submitTrajectory(ros::Publisher &jointTrajectoryCommand_pub)
{
  // rostopic pub -1 /phantomx/trajectory_controller_for_single_step/command trajectory_msgs/JointTrajectory '{joint_names: ["j_tibia_lf", "j_tibia_lm", "j_tibia_lr", "j_tibia_rf", "j_tibia_rm", "j_tibia_rr", "j_thigh_lf", "j_thigh_lm", "j_thigh_lr", "j_thigh_rf", "j_thigh_rm", "j_thigh_rr"], points: [ {positions:[0,0,0,0,0,0,0,0,0,0,0,0], time_from_start: [1,0]}, {positions:[-1,0,-1,0,1,0, 0,0,0,0,0,0], time_from_start: [2,0]}, {positions:[0,0,0,0,0,0,0,0,0,0,0,0], time_from_start: [3,0]}, {positions:[0,-1,0,1,0,1, 0,0,0,0,0,0], time_from_start: [4,0]}, {positions:[0,0,0,0,0,0,0,0,0,0,0,0], time_from_start: [5,0]}, ]}'
  std::cout << "Sending trajectory" << std::endl;
  gaitTrajectory.trajectory.header.stamp = ros::Time::now();
  jointTrajectoryCommand_pub.publish(gaitTrajectory);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "webbie1_gaitTrajectorizer");

  ros::NodeHandle n;
  ros::Publisher jointTrajectoryCommand_pub = n.advertise<control_msgs::FollowJointTrajectoryGoal>("/webbie1/joint_trajectory/command", 1);

  std::cout << "Waiting for topic connections to be ready..." << std::endl;
  // Wait for all topic connections to be ready
  while (0 == jointTrajectoryCommand_pub.getNumSubscribers())
  {
    ROS_INFO("Waiting for subscribers to connect");
    ros::Duration(1.0).sleep();
  }

  ros::Subscriber jointStateSub(n.subscribe("/webbie1/joint_trajectory/state", 1, jointStateCallback2));


  std::cout << "Starting" << std::endl;
  calculateTrajectory();

  while (ros::ok())
  {
    submitTrajectory(jointTrajectoryCommand_pub);

    ros::spin();
    return 0;

  }

  //	ros::spin();

  return 0;
}