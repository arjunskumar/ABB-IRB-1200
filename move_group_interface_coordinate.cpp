#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "move_group_interface");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//Sleep to allow Rviz to come up
	sleep(20.0);

	//----------------------------
	//Setup
	//----------------------------

	//Setting up MoveGroup class with the group to control
	moveit::planning_interface::MoveGroup group("arm");

	//Using :planning_scene_interface:'PlanningSceneInterface' class to deal directly with the world
	moveit::planning_interface::PlanningSceneInterface planning_Scene_interface;

	//(Optional) Create a publisher for visualizing plans in Rviz
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	//-----------------------------
	//Getting Basic Information
	//-----------------------------

	//Print name of reference frame for robot
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
	//Print name of end-effector link for this group
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

	//-----------------------------
	//Planning to a Pose Goal
	//-----------------------------

	//Plan a motion for this group to a desired pose for end-effector
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 0.174627;
	target_pose1.orientation.x = -0.043707;
	target_pose1.orientation.y = 0.789749;
	target_pose1.orientation.z = 0.586423;
	target_pose1.position.x = -0.687822;
	target_pose1.position.y = 0.316510;
	target_pose1.position.z = 0.475076;
	
	group.setPoseTarget(target_pose1);
	group.setGoalTolerance(0.01);
	//Call the planner to compute the plan and visualize it
	//Just planning, not asking move_group to move the robot
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	//Give Rviz time to visualize the plan
	sleep(5.0);
	
	ros::waitForShutdown();
	return 0;
}
