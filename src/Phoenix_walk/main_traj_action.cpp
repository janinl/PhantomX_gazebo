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

void submitTrajectory();

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *webbie1Trajectory;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState &state,
            const control_msgs::FollowJointTrajectoryResultConstPtr &result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  std::cout << "doneCB: " << *result << std::endl;
  if (state == state.PREEMPTED)
    ros::shutdown();
  else
    submitTrajectory();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback)
{
  //return;
  //std::cout << "Feedback: " << *feedback << std::endl;
  //ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
  for (int i = 0; i < feedback->error.positions.size(); ++i)
  {
    if (std::abs(feedback->error.positions[i]) > 0.1)
    {
      std::cout << "Servo struggling: " << i << "\n"
                << *feedback << std::endl;
      webbie1Trajectory->cancelAllGoals();
      break;
    }
  }
}

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
      webbie1Trajectory->cancelAllGoals();
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
  g_InControlState.TravelLength.z = -127;
  g_BodyYOffset = 100;
  g_InControlState.BodyPos.y = 100;
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
  for (int repeat = 0; repeat < 4; ++repeat)
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

void submitTrajectory()
{
  std::cout << "Sending trajectory" << std::endl;
  gaitTrajectory.trajectory.header.stamp = ros::Time::now();
  webbie1Trajectory->sendGoal(gaitTrajectory, &doneCb, &activeCb, &feedbackCb);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_gazebo_message_wrapper");
  calculateTrajectory();

  webbie1Trajectory = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("phantomx/trajectory_controller_for_single_step/follow_joint_trajectory", true);
  std::cout << "Waiting for trajectory action controller..." << std::endl;
  webbie1Trajectory->waitForServer();
  std::cout << "Starting" << std::endl;

  ros::NodeHandle n;
  ros::Subscriber jointStateSub(n.subscribe("/phantomx/joint_states", 1, jointStateCallback2));

  //calculateTrajectory();

  while (ros::ok())
  {
    submitTrajectory();

    ros::spin();
    return 0;

    std::cout << "sleeping..." << std::endl;
    bool done = webbie1Trajectory->waitForResult();
    std::cout << "done: " << done << std::endl;
    auto result = webbie1Trajectory->getResult();
    std::cout << "result: " << *result << std::endl;
    auto state = webbie1Trajectory->getState();
    std::cout << "state: " << state.toString() << std::endl;
    if (state == state.PREEMPTED)
    {
      return 0;
    }

    // point.time_from_start += ros::Duration(5.0);
    //point.time_from_start.sleep();
    //   ros::Duration(10).sleep();
  }

  //	ros::spin();

  return 0;
}