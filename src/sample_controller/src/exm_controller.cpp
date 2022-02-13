#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

namespace exm_controller_ns
{// the next line is bool signature that is responsible for loading the controller and it is call by controller mangr
	class PositionController : public controller_interface :: Controller<hardware_interface::EffortJointInterface>
	{
		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
		{
			std::string my_joint; // we are getting here the joint that we are to control
			if(!n.getParam("joint", my_joint)) //if ros cannot get the parameter of the joint
			{
				ROS_ERROR("could not find joint name");
				return false;   // then dont let it run since it a boolean function
			}
			joint_ = hw->getHandle(my_joint);
			command_ = joint_.getPosition();

			if (!n.getParam("win", win_))
			{
				ROS_ERROR("could not load the parameter value");
				return false;
			}

			sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &PositionController::setCBmnd, this);

			return true;

		}

		void onUpdate(const ros::Time& time, const ros::Duration& period)
		{
			double error = command_ - joint_.getPosition();
			double command_effort = error*win_;
			joint_.setCommand(command_effort);
		}
		void setCBmnd(const std_msgs::Float64ConstPtr& msg)
		{
			command_ = msg->data;
		}
		void starting(const ros::Time& time){}
		void stopping(const ros::Time& time){}
		private:
			hardware_interface::JointHandle joint_;
			double win_;
			double command_;
			ros::Subscriber sub_command_;
	};

	PLUGINLIB_EXPORT_CLASS(exm_controller_ns::PositionController, controller_interface::ControllerBase);

}