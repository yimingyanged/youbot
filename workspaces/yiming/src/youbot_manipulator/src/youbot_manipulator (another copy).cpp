#include "youbot_manipulator/youbot_manipulator.h"
namespace youbot_manipulator
{
	YoubotManipulator::YoubotManipulator(ros::NodeHandle * nh_, std::string ik_ns, std::string plan_ns, std::string gripper_ns, std::string target_ns, std::string base_ns, geometry_msgs::Pose pre_offset, bool has_base) :
			spinner_(2), gripper_ac_(gripper_ns, true), base_ac_(base_ns, true), offset_(pre_offset), has_base_(has_base), ik_group_(ik_ns), plan_group_(plan_ns), ik_ns_(ik_ns), plan_ns_(plan_ns), robot_model_loader("robot_description")
	{
		start_sub_ = nh_->subscribe<std_msgs::Bool>("youbot_manipulator/start", 1, boost::bind(&YoubotManipulator::startCallback, this, _1));
		target_sub_ = nh_->subscribe<geometry_msgs::PoseStamped>(target_ns, 1, boost::bind(&YoubotManipulator::targetCallback, this, _1));
		vel_pub_ = nh_->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
		done_pub_ = nh_->advertise<std_msgs::Bool>("youbot_manipulator/done", 1, true);
		has_gripper_ = gripper_ac_.waitForServer(ros::Duration(1));
		if (!has_gripper_)
			ROS_INFO("Gripper server not connected, gripper command will not be performed");
		if (!has_base_)
		{
			ROS_INFO("has_base set to false, base movement will not be performed");
		}
		else if (!base_ac_.waitForServer(ros::Duration(1)))
		{
			ROS_INFO("Base navigation server not connected, base movement will not be performed");
			has_base_ = false;
		}
		ROS_INFO("YoubotManipulator Initialised, spinner starts");
		has_goal_ = false;
		spinner_.start();
	}

	void YoubotManipulator::setTarget(const geometry_msgs::PoseStamped & target)
	{
		boost::mutex::scoped_lock(lock_);
		if ((has_goal_ && goal_.header.stamp < target.header.stamp) || !has_goal_)
		{
			goal_ = target;
			has_goal_ = true;
		}
	}
	void YoubotManipulator::targetCallback(const geometry_msgs::PoseStamped::ConstPtr & target)
	{
		if (!has_goal_)
			ROS_INFO("GOAL at (%f, %f, %f)", target->pose.position.x, target->pose.position.y, target->pose.position.z);
		boost::mutex::scoped_lock(lock_);
		has_goal_ = true;
		goal_ = *target;
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

	void YoubotManipulator::startCallback(const std_msgs::BoolConstPtr & start)
	{
		if (start->data) /**	Direct grasp	*/
		{
			ROS_INFO("Started with direct grasp");

			bool succeeded = directGrasp();

			if (succeeded)
			{
				ROS_INFO("Grasp succeeded.");
				std_msgs::Bool done;
				done.data = true;
				done_pub_.publish(done);
			}
		}
		else
		{
			ROS_INFO("Invalid starter, or target pose not received");
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
		std::vector<double> lookforward_joints;
		lookforward_joints.resize(8);
		lookforward_joints[0] = 0.0;
		lookforward_joints[1] = 0.0;
		lookforward_joints[2] = 0.0;
		lookforward_joints[3] = 2.9496;
		lookforward_joints[4] = 0.1085;
		lookforward_joints[5] = -1.3352;
		lookforward_joints[6] = 3.1117;
		lookforward_joints[7] = 2.8984;
		/** If dont have goal, search for it */
		if (!has_goal_)
		{
			/** Look forward searching pose, from MoveIt! */

			ik_group_.setJointValueTarget(lookforward_joints);

			bool success;
			ik_group_.asyncMove();

			/** Should see the target now, do circling search if not*/
			if (!has_goal_)
			{
				for (double tmp = 1.4; tmp < 4.5; tmp += 0.2) /** Searching angle range 1.4 - 4.5 */
				{
					lookforward_joints[3] = tmp;
					ik_group_.setJointValueTarget(lookforward_joints);
					ik_group_.asyncMove();
					ros::Duration(2).sleep();
					if (has_goal_)
					{
						ROS_INFO("Goal Detected, leaving searching loop, performing grasp");
						break;
					}
				}
			}

		}

		if (!has_goal_ || goal_.header.stamp.toSec() + 0.5 < ros::Time::now().toSec())
		{
			ROS_INFO("GOAL %f, NOW %f", goal_.header.stamp.toSec(), ros::Time::now().toSec());
			ROS_INFO("Goal Not Detected. Grasp can not be performed");
			return false;
		}

		std::vector<double> joints, pre_joints;

		bool valid = false;
		int cnt = 0, approach = 0;
		while (cnt - approach < 10)
		{
			//geometry_msgs::PoseStamped pre_goal = setToGraspPose(getGoal(), true);
			geometry_msgs::PoseStamped grasp_goal = setToGraspPose(getGoal(), false);

			if (grasp_goal.header.stamp.toSec() + 0.5 < ros::Time::now().toSec())
			{
				ROS_INFO("GOAL %f, NOW %f", grasp_goal.header.stamp.sec, ros::Time::now().sec);
				ROS_INFO("Goal is too old.");
				return false;
			}
			if (!getIK(joints, grasp_goal))
			{
				ROS_INFO("Direct Grasp: IK not found");
				return false;
			}
			double dist_goal = sqrt((getGoal().pose.position.x * getGoal().pose.position.x) + (getGoal().pose.position.y * getGoal().pose.position.y));
			double dist_est = sqrt((joints[0] * joints[0]) + (joints[1] * joints[1]));
			ROS_INFO("GOAL: %f,  Est: %f", dist_goal, dist_est);
			if (dist_goal > dist_est + 0.3 || approach > 0)
			{
				approach++;

				if (approach > 3)
				{
					moveBase(joints, false);
					ros::Duration(2).sleep();
					break;
				}
				else
				{
					moveBase(joints, true);
					ros::Duration(2).sleep();
				}
			}
			cnt++;
		}

//	if (!getIK(pre_joints, pre_goal))
//		{
//			ROS_INFO("Direct Grasp: IK not found for pre grasp");
//			return false;
//		}
		pre_joints.resize(8);
		pre_joints = joints;
		pre_joints[5] = pre_joints[5] - 0.13;
		pre_joints[6] = pre_joints[6] + 0.13;

		ik_group_.setJointValueTarget(pre_joints);
		bool succeeded = ik_group_.plan(plan_);
		if (!succeeded)
		{
			ROS_INFO("Direct Grasp: Arm planning for pre grasp NOT succeeded");
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
		ik_group_.setStartStateToCurrentState();
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
		if (has_gripper_)
		{
			gripper_ac_.sendGoal(graspGripper(), boost::bind(&YoubotManipulator::gripperDoneCallback, this, _1, _2));
			gripper_ac_.waitForResult();
		}

		std::vector<double> lookup_joints;
		lookup_joints.resize(8);
		lookup_joints[0] = 0.0;
		lookup_joints[1] = 0.0;
		lookup_joints[2] = 0.0;
		lookup_joints[3] = 3.0;
		lookup_joints[4] = 1.16;
		lookup_joints[5] = -2.56;
		lookup_joints[6] = 1.82;
		lookup_joints[7] = 2.96;
		ik_group_.setStartStateToCurrentState();
		ik_group_.setJointValueTarget(lookup_joints);
		ik_group_.move();
		ros::Duration(4).sleep();
		std::vector<double> place_joints;
		place_joints.resize(8);
		place_joints[0] = 0.0;
		place_joints[1] = 0.0;
		place_joints[2] = 0.0;
		place_joints[3] = 3.1125;
		place_joints[4] = 0.8111;
		place_joints[5] = -3.2487;
		place_joints[6] = 0.6739;
		place_joints[7] = 0.0;
		ik_group_.setJointValueTarget(place_joints);
		ik_group_.move();
		ros::Duration(4).sleep();
		if (has_gripper_)
		{
			gripper_ac_.sendGoal(openGripper(), boost::bind(&YoubotManipulator::gripperDoneCallback, this, _1, _2));
			gripper_ac_.waitForResult();
		}
		ik_group_.setJointValueTarget(lookforward_joints);
		ik_group_.move();
		ros::Duration(4).sleep();

		if (has_gripper_)
		{
			gripper_ac_.sendGoal(graspGripper(), boost::bind(&YoubotManipulator::gripperDoneCallback, this, _1, _2));
			gripper_ac_.waitForResult();
		}
		return true;
	}

	bool YoubotManipulator::getIK(std::vector<double> & joint_values, geometry_msgs::PoseStamped goal)
	{

		robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
		robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
		//kinematic_state->setToDefaultValues();
		const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(ik_ns_);
		const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for (std::size_t i = 0; i < joint_names.size(); ++i)
		{

			if (joint_values[i] > 10)
			{
				ROS_INFO("Joint %s has a value of %f. Unknown Errors, please restart the node", joint_names[i].c_str(), joint_values[i]);
				return false;
			}
		}

		//const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("moveit_fixed_link");
		bool found_ik = false;

		int cnt = 0;
		while (cnt < 10)
		{
			found_ik = kinematic_state->setFromIK(joint_model_group, goal.pose, 10, 0.1);
			if (found_ik)
			{
				kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
			}
			else
			{
				ROS_INFO("Did not find IK solution");
			}
			cnt++;
		}
		return found_ik;
	}

	bool YoubotManipulator::moveBase(const std::vector<double> & joint_values, bool approach)
	{

		move_base_msgs::MoveBaseGoal base_goal;
		base_goal.target_pose.header.frame_id = "odom";
		base_goal.target_pose.header.stamp = ros::Time::now();
		ROS_INFO("BASE JOINTS: x: %f, y: %f, r: %f", joint_values[0], joint_values[1], joint_values[2]);

		base_goal.target_pose.pose.position.x = joint_values[0];
		base_goal.target_pose.pose.position.y = joint_values[1];
		if (approach)
		{
			base_goal.target_pose.pose.position.x = base_goal.target_pose.pose.position.x * 0.7;
			base_goal.target_pose.pose.position.y = base_goal.target_pose.pose.position.y * 0.7;
		}
		base_goal.target_pose.pose.orientation.w = cos((joint_values[2]) / 2);
		base_goal.target_pose.pose.orientation.z = sin((joint_values[2]) / 2);
		base_goal.target_pose.pose.orientation.w /= (base_goal.target_pose.pose.orientation.w * base_goal.target_pose.pose.orientation.w + base_goal.target_pose.pose.orientation.z * base_goal.target_pose.pose.orientation.z);
		base_goal.target_pose.pose.orientation.z /= (base_goal.target_pose.pose.orientation.w * base_goal.target_pose.pose.orientation.w + base_goal.target_pose.pose.orientation.z * base_goal.target_pose.pose.orientation.z);
		//base_goal.target_pose.pose.orientation.z = -1 * base_goal.target_pose.pose.orientation.z;
		ROS_INFO("Base navigation goal: (%f,%f,%f)", base_goal.target_pose.pose.position.x, base_goal.target_pose.pose.position.y, base_goal.target_pose.pose.position.z);
		ROS_INFO("(%f,%f,%f,%f)", base_goal.target_pose.pose.orientation.x, base_goal.target_pose.pose.orientation.y, base_goal.target_pose.pose.orientation.z, base_goal.target_pose.pose.orientation.w);

		/** Not using Navigation stack */
//		base_goal.target_pose.pose.position.x = base_goal.target_pose.pose.position.x - dist_offset;
//		if (!has_base_)
//			return false;
//		actionlib::SimpleClientGoalState state = base_ac_.sendGoalAndWait(base_goal, ros::Duration(10), ros::Duration(4));
//		if (state.SUCCEEDED)
//			return true;
//		else
//			return false;
		/** Now we using cmd_vel controller */

		double t = 3; /** movement duration */
		ros::Duration base_move(t);
		geometry_msgs::Twist cmd;
		cmd.linear.x = base_goal.target_pose.pose.position.x / t;
		cmd.linear.y = base_goal.target_pose.pose.position.y / t;
		cmd.linear.z = 0;
		cmd.angular.x = 0;
		cmd.angular.y = 0;
		cmd.angular.z = base_goal.target_pose.pose.orientation.z / t;
		vel_pub_.publish(cmd);

		base_move.sleep();
		cmd.linear.x = cmd.linear.y = 0;
		cmd.angular.z = 0;
		vel_pub_.publish(cmd);
		return true;
	}
	geometry_msgs::PoseStamped YoubotManipulator::setToGraspPose(const geometry_msgs::PoseStamped g, bool pre)
	{
		ROS_INFO("Convert GOAL at (%f, %f, %f)", g.pose.position.x, g.pose.position.y, g.pose.position.z);
		ROS_INFO("Orientation (%f, %f, %f, %f)", g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w);
		KDL::Frame grasp = KDL::Frame(KDL::Rotation::Quaternion(g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w), KDL::Vector(g.pose.position.x, g.pose.position.y, g.pose.position.z));
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
} //namespace
