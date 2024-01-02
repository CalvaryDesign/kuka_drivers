// Copyright 2022 Áron Svastits
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <thread>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_publisher.h>
#include "realtime_tools/thread_priority.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(
    executor,
    "controller_manager");

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();

  std::atomic_bool is_configured = false;
  auto is_configured_sub = controller_manager->create_subscription<std_msgs::msg::Bool>(
    "robot_manager/is_configured", qos,
    [&is_configured](std_msgs::msg::Bool::SharedPtr msg) {
      is_configured = msg->data;
    });


  std::thread control_loop([controller_manager, &is_configured]() {
      struct sched_param param;
      param.sched_priority = 99;
      if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        RCLCPP_ERROR(controller_manager->get_logger(), "setscheduler error");
        RCLCPP_ERROR(controller_manager->get_logger(), strerror(errno));
        RCLCPP_WARN(
          controller_manager->get_logger(),
          "You can use the driver but scheduler priority was not set");
      }

      // // publisher for control period
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr period_pub = 
        controller_manager->shared_from_this()->create_publisher<std_msgs::msg::Float64MultiArray>("control_period", rclcpp::SystemDefaultsQoS());

      realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray> rt_pub(period_pub);
      rt_pub.msg_.data.push_back(0);
      rt_pub.msg_.data.push_back(0);
      rt_pub.msg_.data.push_back(0);
      rt_pub.msg_.data.push_back(0);

      const rclcpp::Duration dt =
      rclcpp::Duration::from_seconds(1.0 / controller_manager->get_update_rate());
      std::chrono::milliseconds dt_ms {1000 / controller_manager->get_update_rate()};
      try {
        rclcpp::Time read_time, update_time, write_time, current_time;
        rclcpp::Duration measured_period(0,0);
        // for calculating the measured period of the loop
        rclcpp::Time previous_time = controller_manager->now();
        while (rclcpp::ok()) {
          current_time = controller_manager->now();
          measured_period = current_time - previous_time;
          previous_time = current_time;
          if (is_configured) {
            controller_manager->read(controller_manager->now(), dt);
            read_time = controller_manager->now();
            controller_manager->update(controller_manager->now(), dt);
            update_time = controller_manager->now();
            controller_manager->write(controller_manager->now(), dt);
            write_time = controller_manager->now();

            if(rt_pub.trylock()){
              rt_pub.msg_.data[0] = measured_period.seconds();
              rt_pub.msg_.data[1] = (read_time - current_time).seconds();
              rt_pub.msg_.data[2] = (update_time - read_time).seconds();
              rt_pub.msg_.data[3] = (write_time - update_time).seconds();
              rt_pub.unlockAndPublish();
            }
          } else {
            controller_manager->update(controller_manager->now(), dt);
            std::this_thread::sleep_for(dt_ms);
          }
        }
      } catch (std::exception & e) {
        RCLCPP_ERROR(
          controller_manager->get_logger(), "Quitting control loop due to: %s",
          e.what());
      }
    });

  executor->add_node(controller_manager);

  executor->spin();
  control_loop.join();

  // shutdown
  rclcpp::shutdown();

  return 0;
}
