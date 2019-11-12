#ifndef POSVELACCEFF_CONTROLLERS_POSVELACCEFF_CONTROLLERS_HPP
#define POSVELACCEFF_CONTROLLERS_POSVELACCEFF_CONTROLLERS_HPP

#include <stdexcept>
#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <posvelacceff_command_interface/posvelacceff_command_interface.hpp>

#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

namespace posvelacceff_controllers
{
//
// subscribes std_msgs::Float64MultiArray, grabs PosVelAccEffJointInterface
//

class PosVelAccEffControllerForPosVelAccEffInterface
  : public controller_interface::Controller< posvelacceff_command_interface::PosVelAccEffJointInterface >
{
public:
  typedef posvelacceff_command_interface::PosVelAccEffJointInterface HardwareInterface;

  PosVelAccEffControllerForPosVelAccEffInterface()
  {
  }

  virtual ~PosVelAccEffControllerForPosVelAccEffInterface()
  {
  }

  virtual bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
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

    // start subscribing command
    sub_cmd_ = controller_nh.subscribe< std_msgs::Float64MultiArray >(
        "command", 1, &PosVelAccEffControllerForPosVelAccEffInterface::commandCB, this);

    return true;
  }

  virtual void starting(const ros::Time& time)
  {
    //
    cached_pos_ = boost::none;

    // put default command
    std::vector< double > cmd_def(4, 0.);
    cmd_def[0] = jnt_.getPosition();
    buf_cmd_.writeFromNonRT(cmd_def);
  }

  virtual void update(const ros::Time& time, const ros::Duration& period)
  {
    const std::vector< double >& cmd(*buf_cmd_.readFromRT());

    if (cmd[1] >= 0.)
    {
      cached_pos_ = boost::none;
    }
    else
    {
      if (cached_pos_ == boost::none)
      {
        cached_pos_ = jnt_.getPosition();
      }
    }

    // ad-hoc!!!!! if the profile velocity is negative, ignore the position command
    jnt_.setCommandPosition(cmd[1] >= 0. ? cmd[0] : cached_pos_.get());
    jnt_.setCommandVelocity(cmd[1] >= 0. ? cmd[1] : 0.);
    jnt_.setCommandAcceleration(cmd[2]);
    jnt_.setCommandEffort(cmd[3]);
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
  boost::optional< double > cached_pos_;

  ros::Subscriber sub_cmd_;
  realtime_tools::RealtimeBuffer< std::vector< double > > buf_cmd_;
};

//
// subscribes std_msgs::Float64MultiArray, grabs PositionJointInterface
//

class PosVelAccEffControllerForPositionInterface
  : public controller_interface::Controller< hardware_interface::PositionJointInterface >
{
public:
  typedef hardware_interface::PositionJointInterface HardwareInterface;

  PosVelAccEffControllerForPositionInterface()
  {
  }

  virtual ~PosVelAccEffControllerForPositionInterface()
  {
  }

  virtual bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
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

    // start subscribing command
    sub_cmd_ = controller_nh.subscribe< std_msgs::Float64MultiArray >(
        "command", 1, &PosVelAccEffControllerForPositionInterface::commandCB, this);

    return true;
  }

  virtual void starting(const ros::Time& time)
  {
    // put default command
    std::vector< double > cmd_def(4, 0.);
    cmd_def[0] = jnt_.getPosition();
    buf_cmd_.writeFromNonRT(cmd_def);
  }

  virtual void update(const ros::Time& time, const ros::Duration& period)
  {
    const std::vector< double >& cmd(*buf_cmd_.readFromRT());
    // ad-hoc!!!!! if the profile velocity is negative, ignore the position command
    jnt_.setCommand(cmd[1] >= 0. ? cmd[0] : jnt_.getPosition());
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
  hardware_interface::JointHandle jnt_;

  ros::Subscriber sub_cmd_;
  realtime_tools::RealtimeBuffer< std::vector< double > > buf_cmd_;
};

//
// subscribes std_msgs::Float64MultiArray, grabs any supported hardware interface
//

class PosVelAccEffController : public controller_interface::MultiInterfaceController<
                                   PosVelAccEffControllerForPosVelAccEffInterface::HardwareInterface,
                                   PosVelAccEffControllerForPositionInterface::HardwareInterface >
{
public:
  PosVelAccEffController()
    : controller_interface::MultiInterfaceController< PosVelAccEffControllerForPosVelAccEffInterface::HardwareInterface,
                                                      PosVelAccEffControllerForPositionInterface::HardwareInterface >(
          /* allow_optional_interfaces = */ true)
  {
  }

  virtual ~PosVelAccEffController()
  {
  }

  virtual bool init(hardware_interface::RobotHW* robot, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    if (!robot)
    {
      ROS_ERROR("Null robot hardware");
      return false;
    }

    // try to init backend controllers one by one until the first successful initialization.
    return initBackend< PosVelAccEffControllerForPosVelAccEffInterface >(robot, root_nh, controller_nh) ||
           initBackend< PosVelAccEffControllerForPositionInterface >(robot, root_nh, controller_nh);
  }

  virtual void starting(const ros::Time& time)
  {
    if (backend_)
    {
      backend_->starting(time);
    }
  }

  virtual void update(const ros::Time& time, const ros::Duration& period)
  {
    if (backend_)
    {
      backend_->update(time, period);
    }
  }

  virtual void stopping(const ros::Time& time)
  {
    if (backend_)
    {
      backend_->stopping(time);
    }
  }

private:
  // init a backend controller. return whether the backend has been initialized or not.
  // the initialized backend will be stored to the backend_ member variable.
  template < class Backend >
  bool initBackend(hardware_interface::RobotHW* robot, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    typedef typename Backend::HardwareInterface HardwareInterface;

    // find the hardware interface for the type of the backend controller
    HardwareInterface* hw(robot->get< HardwareInterface >());
    if (!hw)
    {
      return false;
    }

    // init the backend with the hardware interface found
    boost::shared_ptr< Backend > new_backend(new Backend());
    if (!new_backend->init(hw, root_nh, controller_nh))
    {
      return false;
    }

    // store the initialized backend
    backend_ = new_backend;

    return true;
  }

private:
  boost::shared_ptr< controller_interface::ControllerBase > backend_;
};
}  // namespace posvelacceff_controllers

#endif