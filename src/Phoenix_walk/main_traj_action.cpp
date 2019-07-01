#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "sensor_msgs/JointState.h"

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *webbie1Trajectory;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState &state,
            const control_msgs::FollowJointTrajectoryResultConstPtr &result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  std::cout << "doneCB: " << *result << std::endl;
  if (state == state.PREEMPTED)
    ros::shutdown();
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

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_gazebo_message_wrapper");

  webbie1Trajectory = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("phantomx/head_controller/follow_joint_trajectory", true);
  std::cout << "Waiting for trajectory action controller..." << std::endl;
  webbie1Trajectory->waitForServer();
  std::cout << "Starting" << std::endl;

  ros::NodeHandle n;
  ros::Subscriber jointStateSub(n.subscribe("/phantomx/joint_states", 1, jointStateCallback));

  while (ros::ok())
  {
    control_msgs::FollowJointTrajectoryGoal msg;

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    point.velocities = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    point.accelerations = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    point.time_from_start = ros::Duration(2.0);
    msg.trajectory.points.push_back(point);

    //point.positions = {0, 0, 0, 0, 0, 0, 1, 1, 1, -1, -1, -1, 0, 0, 0, 0, 0, 0};
    //point.time_from_start += ros::Duration(2.0);
    //msg.trajectory.points.push_back(point);

    for (int i = 6; i < 12; ++i)
    {
      point.positions[i - 1] = 0;
      point.positions[i] = i < 9 ? -1 : 1;
      point.time_from_start += ros::Duration(1.0);
      msg.trajectory.points.push_back(point);
    }

    msg.trajectory.joint_names = {
        "j_c1_lf", "j_c1_lm", "j_c1_lr", "j_c1_rf", "j_c1_rm", "j_c1_rr",
        "j_thigh_lf", "j_thigh_lm", "j_thigh_lr", "j_thigh_rf", "j_thigh_rm", "j_thigh_rr",
        "j_tibia_lf", "j_tibia_lm", "j_tibia_lr", "j_tibia_rf", "j_tibia_rm", "j_tibia_rr"};

    // fill message header and sent it out
    msg.trajectory.header.frame_id = "whatever";
    msg.trajectory.header.stamp = ros::Time::now();
    std::cout << "Sending goal" << std::endl;
    webbie1Trajectory->sendGoal(msg, &doneCb, &activeCb, &feedbackCb);

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