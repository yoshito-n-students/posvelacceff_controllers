#ifndef POSVELACCEFF_CONTROLLERS_POSITION_CONTROLLER_HPP
#define POSVELACCEFF_CONTROLLERS_POSITION_CONTROLLER_HPP

#include <stdexcept>
#include <string>

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <posvelacceff_command_interface/posvelacceff_command_interface.hpp>

namespace posvelacceff_controllers
{
class PositionController
  : public controller_interface::Controller< posvelacceff_command_interface::PosVelAccEffJointInterface >
{
public:
  PositionController()
  {
  }

  virtual ~PositionController()
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

    jnt_.setCommandVelocity(controller_nh.param("default_velocity_command", 0.));
    jnt_.setCommandAcceleration(controller_nh.param("default_acceleration_command", 0.));
    jnt_.setCommandEffort(controller_nh.param("default_effort_command", 0.));

    buf_cmd_.initRT(jnt_.getPosition());

    sub_cmd_ = controller_nh.subscribe< std_msgs::Float64 >("command", 1, &PositionController::commandCB, this);

    return true;
  }

  virtual void starting(const ros::Time& time)
  {
    // nothing to do
  }

  virtual void update(const ros::Time& time, const ros::Duration& period)
  {
    jnt_.setCommandPosition(*buf_cmd_.readFromRT());
  }

  virtual void stopping(const ros::Time& time)
  {
    // nothing to do
  }

private:
  void commandCB(const std_msgs::Float64ConstPtr& msg)
  {
    buf_cmd_.writeFromNonRT(msg->data);
  }

private:
  posvelacceff_command_interface::PosVelAccEffJointHandle jnt_;

  ros::Subscriber sub_cmd_;
  realtime_tools::RealtimeBuffer< double > buf_cmd_;
};

}  // namespace posvelacceff_controllers

#endif