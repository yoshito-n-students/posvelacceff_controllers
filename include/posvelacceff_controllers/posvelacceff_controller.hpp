#ifndef POSVELACCEFF_CONTROLLERS_POSVELACCEFF_CONTROLLER_HPP
#define POSVELACCEFF_CONTROLLERS_POSVELACCEFF_CONTROLLER_HPP

#include <stdexcept>
#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <posvelacceff_command_interface/posvelacceff_command_interface.hpp>

namespace posvelacceff_controllers
{
class PosVelAccEffController
  : public controller_interface::Controller< posvelacceff_command_interface::PosVelAccEffJointInterface >
{
public:
  PosVelAccEffController()
  {
  }

  virtual ~PosVelAccEffController()
  {
  }

  virtual bool init(posvelacceff_command_interface::PosVelAccEffJointInterface* hw, ros::NodeHandle& root_nh,
                    ros::NodeHandle& controller_nh)
  {
    // grab the joint handle
    if (!hw)
    {
      ROS_ERROR("Null joint interface");
      return false;
    }
    std::string joint_name;
    if (!controller_nh.getParam("joint", joint_name))
    {
      ROS_ERROR("No \"joint\" param");
      return false;
    }
    try
    {
      jnt_ = hw->getHandle(joint_name);
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("No joint handle found with \"" << joint_name << "\"");
      return false;
    }

    // put default command
    std::vector< double > cmd_def(4, 0.);
    cmd_def[0] = jnt_.getPosition();
    buf_cmd_.initRT(cmd_def);

    // start subscribing command
    sub_cmd_ =
        controller_nh.subscribe< std_msgs::Float64MultiArray >("command", 1, &PosVelAccEffController::commandCB, this);

    return true;
  }

  virtual void starting(const ros::Time& time)
  {
    // nothing to do
  }

  virtual void update(const ros::Time& time, const ros::Duration& period)
  {
    const std::vector< double >* const cmd(buf_cmd_.readFromRT());
    jnt_.setCommandPosition((*cmd)[0]);
    jnt_.setCommandVelocity((*cmd)[1]);
    jnt_.setCommandAcceleration((*cmd)[2]);
    jnt_.setCommandEffort((*cmd)[3]);
  }

  virtual void stopping(const ros::Time& time)
  {
    // nothing to do
  }

private:
  void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
    if (msg->data.size() != 4)
    {
      ROS_ERROR("Invalid data size. Skip.");
      return;
    }
    buf_cmd_.writeFromNonRT(msg->data);
  }

private:
  posvelacceff_command_interface::PosVelAccEffJointHandle jnt_;

  ros::Subscriber sub_cmd_;
  realtime_tools::RealtimeBuffer< std::vector< double > > buf_cmd_;
};

}  // namespace posvelacceff_controllers

#endif