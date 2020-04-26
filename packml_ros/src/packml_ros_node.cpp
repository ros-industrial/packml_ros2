// Copyright (c) 2017 Shaun Edwards
// Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific (ROS 2 compatibility)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//


#include <QtCore>

#include <chrono>
#include <thread>
#include <sstream>
#include "packml_sm/state_machine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "packml_msgs/srv/transition.hpp"
#include "packml_msgs/srv/all_status.hpp"
#include "packml_ros/packml_ros.hpp"


int main(int argc, char * argv[])
{
  // Start node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  // Spin all the Qt components as a thread
  std::thread thr(qtWorker, argc, argv);
  while (NULL == QCoreApplication::instance()) {
    printf("Waiting for QCore application to start\n");
  }
  thr.detach();
  SMNode thenode(node);
  rclcpp::spin(node);
  return 0;
}
