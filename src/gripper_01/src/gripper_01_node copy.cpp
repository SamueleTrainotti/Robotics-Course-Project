#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <cmath> 

#include <joint_trajectory_controller/joint_trajectory_msg_utils.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>

using namespace joint_trajectory_controller;
using namespace trajectory_msgs;
using actionlib::SimpleClientGoalState;
using std::string;
using std::vector;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;
typedef std::shared_ptr<ActionClient> ActionClientPtr;
typedef control_msgs::FollowJointTrajectoryGoal ActionGoal;
typedef control_msgs::JointTrajectoryControllerStateConstPtr StateConstPtr;


ActionClientPtr action_client;
double maxErr = 0;

/*class StateFeedback
{
public:
	void manage(const control_msgs::JointTrajectoryControllerStateConstPtr &state)
	{
		std::cout << std::endl
				  << "Desired vector" << std::endl;
		for (auto const &i : state->desired.positions)
		{
			std::cout << i << " ";
		}

		std::cout << std::endl
				  << "Desired vector" << std::endl;
		for (auto const &i : state->actual.positions)
		{
			std::cout << i << " ";
		}

		std::cout << std::endl
				  << "Desired vector" << std::endl;
		for (auto const &i : state->error.positions)
		{
			std::cout << i << " ";
		}
	}
}*/

void stateCB(const StateConstPtr &state){
	const auto& eBegin = state->error.positions.cbegin();
	const auto& eEnd = state->error.positions.cend();

	/*if (std::any_of(eBegin, eEnd, [](double e){return abs(e) > 0.01;}))
	{
		std::cout << "Error, cancelling... " << std::endl;
		action_client->cancelGoal();
	}*/
	
}

void doneCB(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result) {
	ROS_INFO("Finished in state [%s], maxError= %f", state.toString().c_str(), maxErr);
	maxErr = 0;
}
void activeCB() {
	ROS_INFO("Goal just went active");
}
void feedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback) {
	std::cout << "error: ";
	std::cout.precision(3);
	for(const auto& p:feedback->error.positions) {
		std::cout << p << " ";
		if(abs(p) > maxErr)
			maxErr = p;
		if(abs(p) > 0.055)
			action_client->cancelAllGoals();
	}
	std::cout << std::endl;
}

main(int argc, char **argv)
{
	ros::init(argc, argv, "gripper_01_node");
	ros::NodeHandle n;

	int n_joints = 9;
	ActionGoal traj_goal;
	action_client.reset(new ActionClient("/simple_gripper/joint_trajectory_controller/follow_joint_trajectory"));

	vector<JointTrajectoryPoint> open(1);
	vector<JointTrajectoryPoint> close(1);
	trajectory_msgs::JointTrajectory trajectory_open;
	trajectory_msgs::JointTrajectory trajectory_close;
	vector<std::string> joint_name = { "simple_gripper_right_driver_joint" };

	// JOINT value range: 0.00 -> 0.83

	// OPEN movment
	open[0].positions = { 0 };
	open[0].time_from_start = ros::Duration(3.0);

	trajectory_open.header.stamp = ros::Time(0.0);
	trajectory_open.joint_names = joint_name;
	trajectory_open.points = open;

	// CLOSE movment
	close[0].positions = { 0.83 };
	close[0].time_from_start = ros::Duration(3.0);

	trajectory_close.header.stamp = ros::Time(0);
	trajectory_close.joint_names = joint_name;
	trajectory_close.points = close;
	/*
		// TOPICS
		StateFeedback state_feedback;
		ros::Publisher controller = n.advertise<trajectory_msgs::JointTrajectory>("/hand_controller/command", 1);
		// ros::Subscriber state = n.subscribe<control_msgs::JointTrajectoryControllerState>("/hand_controller/state", 1, &StateFeedback::manage, state_feedback);
		ros::Subscriber state = n.subscribe("/hand_controller/state", 1, &StateFeedback::manage, state_feedback);
		*/
	ros::Rate loop_rate(100); // run loop with 100 Hz frequency ( = 0.01 seconds )

	ros::Publisher traj_pub;
	traj_pub = n.advertise<trajectory_msgs::JointTrajectory>("/simple_gripper/joint_trajectory_controller/command", 1);
	
	// State subscriber
	ros::Subscriber gripper_state;
    gripper_state = n.subscribe<control_msgs::JointTrajectoryControllerState>("/simple_gripper/joint_trajectory_controller/state", 1, stateCB);
	bool turn = true;
	
	while (ros::ok())
	{
		std::cout << "Press a key to continue...";
		std::cin.get();

		traj_goal.trajectory = turn ? trajectory_open : trajectory_close;
		traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
		action_client->sendGoal(traj_goal, &doneCB, &activeCB, &feedbackCB);
		action_client->waitForResult();
		turn = !turn;
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	gripper_state.shutdown();
	ros::shutdown();
	return 0;
}
