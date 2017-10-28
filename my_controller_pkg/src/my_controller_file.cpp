//#include "my_controller_pkg/my_controller_file.h"

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace controller_ns{

class PositionController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &nh)
  {
    // get joint name from the parameter server
    std::string my_joint;
    if (!nh.getParam("joint", my_joint)){
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle(my_joint);  // throws on failure
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    //double error = setpoint_ - joint_.getPosition();
    joint_.setCommand(command_);
  }

  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) { }

private:
  hardware_interface::JointHandle joint_;
  static const double gain_ = 1.25;
  static const double setpoint_ = 3.00;
  static const double command_ = 0.02;
};

//PLUGINLIB_DECLARE_CLASS(my_controller_pkg, MyControllerPlugin, controller_ns::PositionController, controller_interface::ControllerBase);

PLUGINLIB_EXPORT_CLASS(controller_ns::PositionController, controller_interface::ControllerBase)

}//namespace
