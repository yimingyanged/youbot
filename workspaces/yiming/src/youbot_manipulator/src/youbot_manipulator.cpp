#include "youbot_manipulator/youbot_manipulator.h"
namespace youbot_manipulator{
YoubotManipulator::YoubotManipulator(ros::NodeHandle * nh_, std::string ik_ns, std::string plan_ns, std::string gripper_ns, std::string target_ns, std::string base_ns, geometry_msgs::Pose pre_offset, bool has_base):
spinner_(2), gripper_ac_(gripper_ns, true), base_ac_(base_ns, true), offset_(pre_offset), has_base_(has_base), ik_group_(ik_ns), plan_group_(plan_ns),
ik_ns_(ik_ns), plan_ns_(plan_ns), robot_model_loader("robot_description")
{
	trigger_sub_ = nh_->subscribe<std_msgs::UInt8>("trigger", 1, boost::bind(&YoubotManipulator::triggerCallback, this, _1));
	target_sub_ = nh_->subscribe<geometry_msgs::PoseStamped>(target_ns, 1, boost::bind(&YoubotManipulator::targetCallback, this, _1));

	has_gripper_ = gripper_ac_.waitForServer(ros::Duration(1));
	if (!has_gripper_)
		ROS_INFO("Gripper server not connected, gripper command will not be performed");
	if(!has_base_)
	{
		ROS_INFO("has_base set to false, base movement will not be performed");
	}
	else if(!base_ac_.waitForServer(ros::Duration(1)))
	{
		ROS_INFO("Base navigation server not connected, base movement will not be performed");
		has_base_ = false;
	}
	ROS_INFO("YoubotManipulator Initialised, spinner starts");
	spinner_.start();
}
void YoubotManipulator::targetCallback(const geometry_msgs::PoseStamped::ConstPtr & target)
{
	if (!has_goal_)
		ROS_INFO("GOAL at (%f, %f, %f)", target->pose.position.x, target->pose.position.y, target->pose.position.z);
	boost::mutex::scoped_lock(lock_);
	has_goal_ = true;
	goal_ = * target;
}
geometry_msgs::PoseStamped YoubotManipulator::getGoal()
{
	boost::mutex::scoped_lock(lock_);
	return goal_;
}
void YoubotManipulator::gripperDoneCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::GripperCommandResult::ConstPtr & result)
{
	if (result->reached_goal)
		ROS_INFO("Gripper command succeeded");
}

void YoubotManipulator::triggerCallback(const std_msgs::UInt8ConstPtr &  trigger)
{
	if ((trigger->data == 0) && has_goal_)	/**	Direct grasp	*/
	{
		ROS_INFO("Triggered with direct grasp");

		bool succeeded = directGrasp();
	}
	else if ((trigger->data == 1) && has_goal_)	/**	With pre-grasp	*/
	{
		ROS_INFO("Triggered with pre-grasp");
	}
	else
	{
		ROS_INFO("Invalid trigger, or target pose not received");
	}
}

control_msgs::GripperCommandGoal YoubotManipulator::openGripper()
{
	ROS_INFO("Gripper: Opening");
	control_msgs::GripperCommandGoal g;
	g.command.max_effort = 2;
	g.command.position = 0.022;
	return g;
}

control_msgs::GripperCommandGoal YoubotManipulator::graspGripper()
{
	ROS_INFO("Gripper: Grasping");
	control_msgs::GripperCommandGoal g;
	g.command.position = 0.004;
	return g;
}

bool YoubotManipulator::directGrasp()
{
	ROS_INFO("Youbot Direct Grasp Manipulator: Starting");
//	std::vector<double> lookup_joints;
//	lookup_joints.resize(8);
//	lookup_joints[0]=0.0;
//	lookup_joints[1]=0.0;
//	lookup_joints[2]=0.0;
//	lookup_joints[3]=3.0;
//	lookup_joints[4]=1.16;
//	lookup_joints[5]=-2.56;
//	lookup_joints[6]=1.82;
//	lookup_joints[7]=2.96;
//	ik_group_.setJointValueTarget(lookup_joints);
//	ik_group_.move();

	geometry_msgs::PoseStamped pre_goal = setToGraspPose(getGoal(), true);
	geometry_msgs::PoseStamped grasp_goal = setToGraspPose(getGoal(), false);
	std::vector<double> joints, pre_joints;
	if (!getIK(joints, grasp_goal))
	{
		ROS_INFO("Direct Grasp: IK not found");
		return false;
	}
	if (!moveBase(joints,0.0))
	{
		ROS_INFO("Direct Grasp:	Base movement NOT succeeded");
		//return false;
	}

//	if (!getIK(pre_joints, pre_goal))
//		{
//			ROS_INFO("Direct Grasp: IK not found for pre grasp");
//			return false;
//		}
	pre_joints.resize(8);
	pre_joints=joints;
	pre_joints[5]=pre_joints[5] - 0.13;
	pre_joints[6]=pre_joints[6] + 0.13;

	ik_group_.setJointValueTarget(pre_joints);
	bool succeeded = ik_group_.plan(plan_);
	if (!succeeded)
	{
		ROS_INFO("Direct Grasp: Arm planning fo pre grasp NOT succeeded");
		return false;
	}
	if (!ik_group_.move())
		{
			ROS_INFO("Pre grasp execute failed");
			return false;
		}
	if (has_gripper_)
	{
		gripper_ac_.sendGoal(openGripper(), boost::bind(&YoubotManipulator::gripperDoneCallback, this, _1, _2));
		gripper_ac_.waitForResult();
	}


	ik_group_.setJointValueTarget(joints);
	succeeded = ik_group_.plan(plan_);
	if (!succeeded)
	{
		ROS_INFO("Direct Grasp: Arm planning for grasp NOT succeeded");
		return false;
	}
	if (!ik_group_.move())
	{
		ROS_INFO("Grasp execute failed");
		return false;
	}
	ros::Duration(1.0).sleep();
	if (has_gripper_){
		gripper_ac_.sendGoal(graspGripper(), boost::bind(&YoubotManipulator::gripperDoneCallback, this, _1, _2));
		gripper_ac_.waitForResult();
	}

		std::vector<double> lookup_joints;
		lookup_joints.resize(8);
		lookup_joints[0]=0.0;
		lookup_joints[1]=0.0;
		lookup_joints[2]=0.0;
		lookup_joints[3]=3.0;
		lookup_joints[4]=1.16;
		lookup_joints[5]=-2.56;
		lookup_joints[6]=1.82;
		lookup_joints[7]=2.96;
		ik_group_.setJointValueTarget(lookup_joints);
		ik_group_.move();
		//ros::Duration()
		std::vector<double> place_joints;
		place_joints.resize(8);
		place_joints[0]=0.0;
		place_joints[1]=0.0;
		place_joints[2]=0.0;
		place_joints[3]=2.8844;
		place_joints[4]=0.5037;
		place_joints[5]=-2.9916;
		place_joints[6]=0.6739;
		place_joints[7]=0.0;
		ik_group_.setJointValueTarget(lookup_joints);
		ik_group_.move();

		if (has_gripper_){
			gripper_ac_.sendGoal(openGripper(), boost::bind(&YoubotManipulator::gripperDoneCallback, this, _1, _2));
			gripper_ac_.waitForResult();
		}
	return true;
}

bool YoubotManipulator::getIK(std::vector<double> & joint_values, geometry_msgs::PoseStamped goal)
{

	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(ik_ns_);
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	for(std::size_t i = 0; i < joint_names.size(); ++i)
	{
	  ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}
	//const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("moveit_fixed_link");
	bool found_ik = kinematic_state->setFromIK(joint_model_group, goal.pose, 10, 0.1);
	if (found_ik)
	{
	  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	  for(std::size_t i=0; i < joint_names.size(); ++i)
	  {
	    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	  }
	}
	else
	{
	  ROS_INFO("Did not find IK solution");
	}
	return found_ik;
}

bool YoubotManipulator::moveBase(const std::vector<double> & joint_values, double dist_offset)
{
	if (!has_base_)
		return false;
	move_base_msgs::MoveBaseGoal base_goal;
	base_goal.target_pose.header.frame_id = "odom";
	base_goal.target_pose.header.stamp = ros::Time::now();
	ROS_INFO("BASE JOINTS: x: %f, y: %f, r: %f", joint_values[0], joint_values[1],joint_values[2]);

	base_goal.target_pose.pose.position.x = joint_values[0];
	base_goal.target_pose.pose.position.y = joint_values[1];
	base_goal.target_pose.pose.orientation.w = cos((joint_values[2])/2);
	base_goal.target_pose.pose.orientation.z = sin((joint_values[2])/2);
	base_goal.target_pose.pose.orientation.w /= (base_goal.target_pose.pose.orientation.w*base_goal.target_pose.pose.orientation.w + base_goal.target_pose.pose.orientation.z*base_goal.target_pose.pose.orientation.z);
	base_goal.target_pose.pose.orientation.z /= (base_goal.target_pose.pose.orientation.w*base_goal.target_pose.pose.orientation.w + base_goal.target_pose.pose.orientation.z*base_goal.target_pose.pose.orientation.z);
	base_goal.target_pose.pose.orientation.z = -1 * base_goal.target_pose.pose.orientation.z;
	ROS_INFO("Base navigation goal: (%f,%f,%f)",base_goal.target_pose.pose.position.x, base_goal.target_pose.pose.position.y, base_goal.target_pose.pose.position.z);
	ROS_INFO("(%f,%f,%f,%f)",base_goal.target_pose.pose.orientation.x, base_goal.target_pose.pose.orientation.y, base_goal.target_pose.pose.orientation.z, base_goal.target_pose.pose.orientation.w);

	base_goal.target_pose.pose.position.x = base_goal.target_pose.pose.position.x - dist_offset;
	ROS_INFO("Press (m) to execute, other keys to exit");
	if (std::getchar() != 'm')
		return false;
	else
		std::getchar();

	actionlib::SimpleClientGoalState state = base_ac_.sendGoalAndWait(base_goal, ros::Duration(10),ros::Duration(4));
	if (state.SUCCEEDED)
		return true;
	else
		return false;
}
geometry_msgs::PoseStamped YoubotManipulator::setToGraspPose(const geometry_msgs::PoseStamped g, bool pre)
{
	ROS_INFO("Convert GOAL at (%f, %f, %f)", g.pose.position.x, g.pose.position.y, g.pose.position.z);
	ROS_INFO("GRASP GOAL O (%f, %f, %f, %f)", g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w);
	KDL::Frame grasp = KDL::Frame(KDL::Rotation::Quaternion(g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w),
								KDL::Vector(g.pose.position.x, g.pose.position.y, g.pose.position.z));
	geometry_msgs::PoseStamped grasp_pose;
	grasp.M.DoRotY(-M_PI);
	grasp.p.z(grasp.p.z() - 0.096);
	grasp.p.x(grasp.p.x() - 0.04);
	//grasp.M.DoRotX(-M_PI/2.0);
	grasp_pose.header = g.header;
	grasp_pose.pose.position.x = grasp.p.x();
	grasp_pose.pose.position.y = grasp.p.y();
	grasp_pose.pose.position.z = grasp.p.z();

	grasp.M.GetQuaternion(grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y, grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w);

	if (pre)
	{
		grasp_pose.pose.position.z = grasp_pose.pose.position.z - 0.03;
	}
	return grasp_pose;
}
}//namespace
