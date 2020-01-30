/**
 * @license Software License Agreement (Apache License)
 *
 * @copyright Copyright (c) 2017 Austin Deric
 * @copyright Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific (ROS 2 compatibility)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <QtGui>
#include <QtCore>
#include <QtWidgets>
#include "ui_packml.h"  // UI layout and components
#include "rclcpp/rclcpp.hpp" // ROS 2 node
#include "packml_msgs/srv/transition.hpp"  // Datatypes for packml topics and services
#include "packml_msgs/msg/state.hpp"
#include "packml_msgs/msg/status.hpp"
#include "packml_msgs/msg/all_times.hpp"
#include "packml_msgs/msg/all_status.hpp"
#include "packml_msgs/srv/all_status.hpp"
#include "std_msgs/msg/int8.hpp"

class PackmlWidget : public QWidget
{
  Q_OBJECT
public:
  PackmlWidget(QWidget* parent = 0);
  virtual ~PackmlWidget();
  std::shared_ptr<rclcpp::Node> nh_;
// UI functions
  Ui::PackmlPanel* ui_;
  //void updateButtonState(const std::shared_ptr<packml_msgs::msg::AllStatus> msg);
  void updateButtonState(std::shared_ptr<packml_msgs::srv::AllStatus::Response> msg);
  void disableAllButtons();
// Communication with PLC driver
  rclcpp::Client<packml_msgs::srv::Transition>::SharedPtr transition_client_;  // Type is Transition
  rclcpp::Client<packml_msgs::srv::AllStatus>::SharedPtr status_client_;
  void callbackTime( std::shared_ptr<packml_msgs::srv::AllStatus::Response> msg);
// Control of the state machine from packml_ros_node via services on pressing the buttons
public Q_SLOTS:
  void onStartButton();
  void onAbortButton();
  void onClearButton();
  void onHoldButton();
  void onResetButton();
  void onUnsuspendButton();
  void onUnholdButton();
  void onSuspendButton();
  void onStopButton();
  void statusRequest();
};
