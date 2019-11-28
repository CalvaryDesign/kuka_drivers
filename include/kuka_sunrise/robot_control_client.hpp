/*
 * RobotControlClient.hpp
 *
 *  Created on: Nov 5, 2019
 *      Author: rosdeveloper
 */

#ifndef INCLUDE_KUKA_SUNRISE_ROBOT_CONTROL_CLIENT_HPP_
#define INCLUDE_KUKA_SUNRISE_ROBOT_CONTROL_CLIENT_HPP_

#include "fri_client/friLBRClient.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "kuka_sunrise_interfaces/srv/set_int.hpp"

#include "kuka_sunrise/internal/activatable_interface.hpp"

#include <condition_variable>
#include <memory>

namespace kuka_sunrise
{

class RobotObserver;
class RobotCommander;

class RobotControlClient : public KUKA::FRI::LBRClient, public ActivatableInterface
{
public:
  RobotControlClient(rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node_);
  ~RobotControlClient();
  bool activate();
  bool deactivate();
  bool setReceiveMultiplier(int receive_multiplier);

  //virtual void onStateChanged(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);
  virtual void monitor();
  virtual void waitForCommand();
  virtual void command();

private:
  std::unique_ptr<RobotObserver> robot_observer_;
  std::unique_ptr<RobotCommander> robot_commander_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr robot_control_node_;
  rclcpp::Service<kuka_sunrise_interfaces::srv::SetInt>::SharedPtr set_receive_multiplier_service_;
  rclcpp::Clock ros_clock_;
  int receive_multiplier_;
  int receive_counter_;

};

}

#endif /* INCLUDE_KUKA_SUNRISE_ROBOT_CONTROL_CLIENT_HPP_ */
