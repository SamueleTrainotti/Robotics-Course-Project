#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
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
int prevErr = 0.1;
double initErr = 0;

// gripper state
ros::Publisher pub_state;

void doneCB(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result) {
	ROS_INFO("Finished in state [%s], maxError= %f", state.toString().c_str(), maxErr);
	std_msgs::Bool b;
	b.data = true;
	pub_state.publish(b);
	maxErr = 0;
}
void activeCB() {
	maxErr = 0;
	ROS_INFO("Goal just went active");
}
void feedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback) {
	std::cout << "error: ";
	std::cout.precision(3);
	
	for(const auto& p:feedback->error.positions) {
		std::cout << p << " ";
		if(abs(p) > maxErr)
			maxErr = p;

		if(abs(p) > (prevErr + 0.0001) && (abs(p) > 0.0005)) {
			action_client->cancelAllGoals();
		}			
		else
			prevErr = abs(p);
		
	}
	
	std::cout << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gripper_01_node");
	ros::NodeHandle n, listener;

	int n_joints = 9;
	ActionGoal traj_goal;
	action_client.reset(new ActionClient("/gripper_trajectory_controller/follow_joint_trajectory"));

	vector<JointTrajectoryPoint> open(1);
	vector<JointTrajectoryPoint> close(1);
	trajectory_msgs::JointTrajectory trajectory_open;
	trajectory_msgs::JointTrajectory trajectory_close;
	vector<std::string> joint_name = { "gripper_right_driver_joint" };

	// JOINT value range: 0.00 -> 0.83

	// OPEN movment
	open[0].positions = { 0 };
	open[0].time_from_start = ros::Duration(2.0);

	trajectory_open.header.stamp = ros::Time(0.0);
	trajectory_open.joint_names = joint_name;
	trajectory_open.points = open;

	// CLOSE movment
	close[0].positions = { 0.83 };
	close[0].time_from_start = ros::Duration(2.0);

	trajectory_close.header.stamp = ros::Time(0);
	trajectory_close.joint_names = joint_name;
	trajectory_close.points = close;
	
	ros::Rate loop_rate(1000); // run loop with 100 Hz frequency ( = 0.01 seconds )

	// move joints
	ros::Publisher traj_pub;
	traj_pub = n.advertise<trajectory_msgs::JointTrajectory>("/gripper_trajectory_controller/command", 1);

	// command listener
	ros::Subscriber commands;
    commands = listener.subscribe<std_msgs::Bool>("gripper_commands", 1, [traj_goal, trajectory_open, trajectory_close, loop_rate](const std_msgs::Bool::ConstPtr& msg) mutable{
		bool data = msg->data;
		// false -> close, true -> open

		traj_goal.trajectory = data ? trajectory_open : trajectory_close;
		traj_goal.trajectory.header.stamp = ros::Time(0); // Start immediately
		action_client->sendGoal(traj_goal, &doneCB, &activeCB, data ? NULL : &feedbackCB);
		action_client->waitForResult();
		loop_rate.sleep();
	});
	// Send gripper state
	pub_state = n.advertise<std_msgs::Bool>("/gripper_state", 1);

	
	ros::spin();

	ros::shutdown();
	return 0;
}
