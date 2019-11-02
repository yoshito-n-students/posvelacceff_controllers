#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

#include "posvelacceff_controllers/position_controller.hpp"
#include "posvelacceff_controllers/posvelacceff_controllers.hpp"

PLUGINLIB_EXPORT_CLASS(posvelacceff_controllers::PosVelAccEffControllerForPosVelAccEffInterface,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(posvelacceff_controllers::PosVelAccEffControllerForPositionInterface,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(posvelacceff_controllers::PosVelAccEffController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(posvelacceff_controllers::PositionController, controller_interface::ControllerBase);