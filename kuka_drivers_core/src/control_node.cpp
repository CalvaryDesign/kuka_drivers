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
#include <sys/sysinfo.h>
#include <iostream>
#include <fstream>
#include <sys/sysinfo.h>
#include <sched.h>
#include <string>
#include <vector>


#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_publisher.h>
#include "realtime_tools/thread_priority.hpp"
int pick_cpu_core(){
 int num_processes = 0;
 int num_processes_prev = 0;

    // Get CPU affinity mask of the current process
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    if (sched_getaffinity(0, sizeof(cpuset), &cpuset) == -1) {
        perror("sched_getaffinity");
        return -1;
    }
    
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("test"), "num cpus: " << CPU_COUNT(&cpuset));
    int target_core = 0;
    for(int core_id = 0; core_id < CPU_COUNT(&cpuset); core_id++){
      // Check if the core is in the affinity mask
      if (!CPU_ISSET(core_id, &cpuset)) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("test"), "Cannot measure processes on core " <<
               core_id << " because it's not in the process affinity mask.");
        return -1;
      }

      // Parse /proc/stat for process information
      std::ifstream proc_stat("/proc/stat");
      std::string line;
      while (std::getline(proc_stat, line)) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "line: " << line);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "looking for: " << "cpu" + std::to_string(core_id));

        if(line.find("cpu" + std::to_string(core_id)) == 0){
          RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "THIS IS THE CRITIAL LINE FOR CORE" << core_id << ": " << line);
          // now we need to find the next space, after which there will be a number.

          std::stringstream ss(line);
          std::string word;

          // Get the second word (which is the first number)
          ss >> word; // throw away the first word
          ss >> word; // grab the second word! The number of user-space processes.
          RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "The word is: " << word);
          // Convert the word to an integer
          num_processes = std::stoi(word);

          if(num_processes < num_processes_prev){
            target_core = core_id;
          }
          num_processes_prev = num_processes;
          // RCLCPP_INFO_STREAM(rclcpp::get_logger("test"), "The isolated number is: " << isolated_number);

          // std::cout << "The isolated number is: " << isolated_number << std::endl;

        }
          // if (line.find("cpu") == 0 && line.find(" " + std::to_string(core_id) + " ") != std::string::npos) {
          //     // Found a process on the target core
          //     num_processes++;
          // }
      }
      // if(num_processes == 0){
      //   return core_id;
      // }
      // num_processes = 0;
    }

    return target_core;
}

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
      param.sched_priority = sched_get_priority_max(SCHED_FIFO);
      pthread_t this_thread = pthread_self();

      if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        RCLCPP_ERROR_STREAM(controller_manager->get_logger(),
          "You can use the driver but scheduler priority was not set. The error is: " << strerror(errno));
      }else{
        // auto pid
        int policy = sched_getscheduler(0);
        RCLCPP_WARN_STREAM(controller_manager->get_logger(), "Setscheduler: " << strerror(errno));
        // Check the new priority and scheduling policy
        sched_getparam(this_thread, &param);
        RCLCPP_WARN_STREAM(controller_manager->get_logger(),
          "New Scheduling Policy: " << policy << std::endl <<
          "New Priority: " << param.sched_priority);
      }

      int core_id = pick_cpu_core();
      if(core_id >= 0){
        // set the affinity to the returned core!
        RCLCPP_WARN_STREAM(controller_manager->get_logger(), "Setting the CPU affinity to " << core_id);
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core_id, &cpuset);

        int s = pthread_setaffinity_np(this_thread, sizeof(cpu_set_t), &cpuset);
        if (s != 0)
            RCLCPP_ERROR_STREAM(controller_manager->get_logger(), "Could not set affinity!");

        /* Check the actual affinity mask assigned to the thread */

        s = pthread_getaffinity_np(this_thread, sizeof(cpu_set_t), &cpuset);
        if (s != 0)
            RCLCPP_ERROR_STREAM(controller_manager->get_logger(), "Could not GET affinity!");

        printf("Set returned by pthread_getaffinity_np() contained:\n");
        for (int j = 0; j < CPU_SETSIZE; j++)
            if (CPU_ISSET(j, &cpuset))
                printf("    CPU %d\n", j);

      }

      // return whether fake hardware is used
      bool fake_hardware = (controller_manager->get_parameter("use_fake_hardware")).as_bool();
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
            if(fake_hardware){
              std::this_thread::sleep_for(dt_ms);
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
