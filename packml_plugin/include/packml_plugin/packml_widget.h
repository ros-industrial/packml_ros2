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
#include "packml_msgs/srv/all_status.hpp"




/**
 * @brief This class creates a Widget (GUI) for an RViz plugin, to control and monitor a 
 * PackML state machine
 */
class PackmlWidget : public QWidget
{
  Q_OBJECT
public:


  /**
  * @brief Constructor of the class that creates the GUI plugin in Rviz, inherits a Widget
  */
  PackmlWidget(QWidget* parent = 0);


  /**
  * @brief Destructor for the widget object
  */
  virtual ~PackmlWidget();

 
  /** 
  * @brief Node for the Widget or RViz plugin
  */
  std::shared_ptr<rclcpp::Node> nh_;


  /**
  * @brief Creates the GUI layout with an UI object as reference
  */
  Ui::PackmlPanel* ui_;


  /**
  * @brief Function called indirectly from a QTimer to update the status of the state machine and enable
  * or disable the buttons that can be pressed from that state according to the standard
  * PackML state machine description
  * @param msg - Response message from an AllStatus service
  */
  void updateButtonState(std::shared_ptr<packml_msgs::srv::AllStatus::Response> msg);


  /**
  * @brief Function that disables all buttons, used by the button update state function
  */
  void disableAllButtons();


  /**
  * @brief Client to request transitions on the state machine from the real PLC
  */
  rclcpp::Client<packml_msgs::srv::Transition>::SharedPtr transition_client_;


  /**
  * @brief Client to request an update on the real PLC's state
  */
  rclcpp::Client<packml_msgs::srv::AllStatus>::SharedPtr status_client_;

  
  /**
  * @brief Function called indirectly from a QTimer to update the elapsed time of the state machine 
  * @param msg - Response message from an AllStatus service
  */ 
  void callbackTime( std::shared_ptr<packml_msgs::srv::AllStatus::Response> msg);
public Q_SLOTS:


  /** 
  * @brief Function that calls for a Transition request in the state machine when the button is pressed 
  */
  void onStartButton();


  /** 
  * @brief Function that calls for a Transition request in the state machine when the button is pressed 
  */
  void onAbortButton();


  /** 
  * @brief Function that calls for a Transition request in the state machine when the button is pressed 
  */
  void onClearButton();


  /**
  * @brief Function that calls for a Transition request in the state machine when the button is pressed 
  */
  void onHoldButton();


  /** 
  * @brief Function that calls for a Transition request in the state machine when the button is pressed 
  */
  void onResetButton();


  /** 
  * @brief Function that calls for a Transition request in the state machine when the button is pressed 
  */
  void onUnsuspendButton();


  /** 
  * @brief Function that calls for a Transition request in the state machine when the button is pressed 
  */
  void onUnholdButton();


  /** 
  * @brief Function that calls for a Transition request in the state machine when the button is pressed 
  */
  void onSuspendButton();


  /** 
  * @brief Function that calls for a Transition request in the state machine when the button is pressed 
  */
  void onStopButton();


  /** 
  * @brief Function called by a QTimer that triggers the update of the state machine state and elapsed time 
  */
  void statusRequest();
};
