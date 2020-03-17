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


#include <iomanip>
#include <sstream>
#include "packml_plugin/packml_widget.h"
#include "rclcpp/rclcpp.hpp"
#include "packml_msgs/srv/transition.hpp"  // Datatypes for services
#include "packml_msgs/srv/all_status.hpp"

PackmlWidget::PackmlWidget(QWidget* parent) : QWidget(parent)
{
  nh_ = rclcpp::Node::make_shared("uiPlugin");  // Creates a node
  // UI setup
  ui_ = new Ui::PackmlPanel;
  ui_->setupUi(this);
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(statusRequest()));
  timer->start(200);
  connect(ui_->start_button, SIGNAL(clicked()), this, SLOT(onStartButton()));
  connect(ui_->abort_button, SIGNAL(clicked()), this, SLOT(onAbortButton()));
  connect(ui_->clear_button, SIGNAL(clicked()), this, SLOT(onClearButton()));
  connect(ui_->hold_button, SIGNAL(clicked()), this, SLOT(onHoldButton()));
  connect(ui_->reset_button, SIGNAL(clicked()), this, SLOT(onResetButton()));
  connect(ui_->unsuspend_button, SIGNAL(clicked()), this, SLOT(onUnsuspendButton()));
  connect(ui_->unhold_button, SIGNAL(clicked()), this, SLOT(onUnholdButton()));
  connect(ui_->suspend_button, SIGNAL(clicked()), this, SLOT(onSuspendButton()));
  connect(ui_->stop_button, SIGNAL(clicked()), this, SLOT(onStopButton()));
  // Interface with packml_ros_node simulator and PLC driver
  transition_client_ = nh_->create_client<packml_msgs::srv::Transition>("transition"); 
  status_client_ = nh_->create_client<packml_msgs::srv::AllStatus>("allStatus");
}

PackmlWidget::~PackmlWidget()
{
  delete ui_;
}


void PackmlWidget::statusRequest()
{
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  if (rclcpp::spin_until_future_complete(nh_, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
    printf("Error in service call, check that the ROS node or PLC driver are up and running\n");
  } else {
    auto result = result_future.get();
    updateButtonState(result);
    callbackTime(result);
  }
}

void PackmlWidget::onStartButton()
{
  auto trans = std::make_shared<packml_msgs::srv::Transition::Request>();
  trans->command = 2;
  transition_client_->async_send_request(trans);
}

void PackmlWidget::onAbortButton()
{
  auto trans = std::make_shared<packml_msgs::srv::Transition::Request>();
  trans->command = 5;
  transition_client_->async_send_request(trans);
}

void PackmlWidget::onClearButton()
{
  auto trans = std::make_shared<packml_msgs::srv::Transition::Request>();
  trans->command = 1;
  transition_client_->async_send_request(trans);
}

void PackmlWidget::onHoldButton()
{
  auto trans = std::make_shared<packml_msgs::srv::Transition::Request>();
  trans->command = 4;
  transition_client_->async_send_request(trans);
}

void PackmlWidget::onResetButton()
{
  auto trans = std::make_shared<packml_msgs::srv::Transition::Request>();
  trans->command = 6;
  transition_client_->async_send_request(trans);
}

void PackmlWidget::onUnsuspendButton()
{
  auto trans = std::make_shared<packml_msgs::srv::Transition::Request>();
  trans->command = 101;
  transition_client_->async_send_request(trans);
}

void PackmlWidget::onUnholdButton()
{
  auto trans = std::make_shared<packml_msgs::srv::Transition::Request>();
  trans->command = 102;
  transition_client_->async_send_request(trans);
}

void PackmlWidget::onSuspendButton()
{
  auto trans = std::make_shared<packml_msgs::srv::Transition::Request>();
  trans->command = 100;
  transition_client_->async_send_request(trans);
}

void PackmlWidget::onStopButton()
{
  auto trans = std::make_shared<packml_msgs::srv::Transition::Request>();
  trans->command = 3;
  transition_client_->async_send_request(trans);
}

void PackmlWidget::updateButtonState(std::shared_ptr<packml_msgs::srv::AllStatus::Response> msg)
{
  disableAllButtons();
  if (msg->stopped_state == true)
  {
    ui_->stopped_state->setStyleSheet("QLabel { color : red; }");
    ui_->abort_button->setEnabled(true);
    ui_->reset_button->setEnabled(true);
  } else {
    ui_->stopped_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->idle_state == true)
  {		
    ui_->idle_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
    ui_->start_button->setEnabled(true);
  } else {
    ui_->idle_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->starting_state == true)
  {
    ui_->starting_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
  } else {
    ui_->starting_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->execute_state == true)
  {
    ui_->execute_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
    ui_->hold_button->setEnabled(true);
    ui_->suspend_button->setEnabled(true);
  } else {
    ui_->execute_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->completing_state == true)
  {
    ui_->completing_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
  } else {
    ui_->completing_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->complete_state == true)
  {
    ui_->complete_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
    ui_->reset_button->setEnabled(true);
  } else {
    ui_->complete_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->suspended_state == true)
  {
    ui_->suspended_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
    ui_->unsuspend_button->setEnabled(true);
  } else {
    ui_->suspended_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->aborting_state == true)
  {
    ui_->aborting_state->setStyleSheet("QLabel { color : red; }");
  } else {
    ui_->aborting_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->aborted_state == true)
  {
    ui_->aborted_state->setStyleSheet("QLabel { color : red; }");
    ui_->clear_button->setEnabled(true);
  } else {
    ui_->aborted_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->holding_state == true)
  {
    ui_->holding_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
  } else {
    ui_->holding_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->held_state == true)
  {
    ui_->held_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
    ui_->unhold_button->setEnabled(true);
  } else {
    ui_->held_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->unholding_state == true)
  {
    ui_->unholding_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
  } else {
    ui_->unholding_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->suspending_state == true)
  {		
    ui_->suspending_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
  } else {
    ui_->suspending_state->setStyleSheet("QLabel { color : black; }"); 
  }
  if (msg->unsuspending_state == true)
  {
    ui_->unsuspending_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
  } else {
    ui_->unsuspending_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->resetting_state == true)
  {
    ui_->resetting_state->setStyleSheet("QLabel { color : red; }");
    ui_->stop_button->setEnabled(true);
    ui_->abort_button->setEnabled(true);
  } else {
    ui_->resetting_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->clearing_state == true)
  {
    ui_->clearing_state->setStyleSheet("QLabel { color : red; }");
    ui_->abort_button->setEnabled(true);
  } else {
    ui_->clearing_state->setStyleSheet("QLabel { color : black; }");
  }
  if (msg->stopping_state == true)
  {
    ui_->stopping_state->setStyleSheet("QLabel { color : red; }");
    ui_->abort_button->setEnabled(true);
  } else {
    ui_->stopping_state->setStyleSheet("QLabel { color : black; }");
  } 
}

void PackmlWidget::disableAllButtons()
{
  ui_->abort_button->setEnabled(false);
  ui_->clear_button->setEnabled(false);
  ui_->hold_button->setEnabled(false);
  ui_->reset_button->setEnabled(false);
  ui_->start_button->setEnabled(false);
  ui_->stop_button->setEnabled(false);
  ui_->suspend_button->setEnabled(false);
  ui_->unhold_button->setEnabled(false);
  ui_->unsuspend_button->setEnabled(false);
}

void PackmlWidget::callbackTime(std::shared_ptr<packml_msgs::srv::AllStatus::Response> msg)
{ 
  std::stringstream stream0;
  stream0 << std::fixed << std::setprecision(2) << msg->t_stopped_state;
  std::string str_stopped = stream0.str();
  ui_->stopped_state->setText(QString::fromStdString("Stopped: " + str_stopped +"s")); 
  std::stringstream stream1;
  stream1 << std::fixed << std::setprecision(2) << msg->t_idle_state;
  std::string str_idle = stream1.str();
  ui_->idle_state->setText(QString::fromStdString("Idle: " + str_idle +"s")); 
  std::stringstream stream2;
  stream2 << std::fixed << std::setprecision(2) << msg->t_starting_state;
  std::string str_starting = stream2.str();
  ui_->starting_state->setText(QString::fromStdString("Starting: " + str_starting +"s")); 
  std::stringstream stream3;
  stream3 << std::fixed << std::setprecision(2) << msg->t_execute_state;
  std::string str_execute = stream3.str();
  ui_->execute_state->setText(QString::fromStdString("Executing: " + str_execute +"s")); 
  std::stringstream stream4;
  stream4 << std::fixed << std::setprecision(2) << msg->t_completing_state;
  std::string str_completing = stream4.str();
  ui_->completing_state->setText(QString::fromStdString("Completing: " + str_completing +"s")); 
  std::stringstream stream5;
  stream5 << std::fixed << std::setprecision(2) << msg->complete_state;
  std::string str_complete = stream5.str();
  ui_->complete_state->setText(QString::fromStdString("Complete: " + str_complete +"s")); 
  std::stringstream stream6;
  stream6 << std::fixed << std::setprecision(2) << msg->t_suspended_state;
  std::string str_suspended = stream6.str();
  ui_->suspended_state->setText(QString::fromStdString("Suspended: " + str_suspended +"s")); 
  std::stringstream stream7;
  stream7 << std::fixed << std::setprecision(2) << msg->t_aborting_state;
  std::string str_aborting = stream7.str();
  ui_->aborting_state->setText(QString::fromStdString("Aborting: " + str_aborting +"s")); 
  std::stringstream stream8;
  stream8 << std::fixed << std::setprecision(2) << msg->t_aborted_state;
  std::string str_aborted= stream8.str();
  ui_->aborted_state->setText(QString::fromStdString("Aborted: " + str_aborted +"s")); 
  std::stringstream stream9;
  stream9 << std::fixed << std::setprecision(2) << msg->t_holding_state;
  std::string str_holding= stream9.str();
  ui_->holding_state->setText(QString::fromStdString("Holding: " + str_holding +"s"));
  std::stringstream stream10;
  stream10 << std::fixed << std::setprecision(2) << msg->t_held_state;
  std::string str_held = stream10.str();
  ui_->held_state->setText(QString::fromStdString("Held: " + str_held +"s"));  
  std::stringstream stream11;
  stream11 << std::fixed << std::setprecision(2) << msg->t_unholding_state;
  std::string str_unholding = stream11.str();
  ui_->unholding_state->setText(QString::fromStdString("Unholding: " + str_unholding +"s")); 
  std::stringstream stream12;
  stream12 << std::fixed << std::setprecision(2) << msg->t_suspending_state;
  std::string str_suspending = stream12.str();;
  ui_->suspending_state->setText(QString::fromStdString("Suspending: " + str_suspending +"s")); 
  std::stringstream stream13;
  stream13 << std::fixed << std::setprecision(2) << msg->t_unsuspending_state;
  std::string str_unsuspending = stream13.str();
  ui_->unsuspending_state->setText(QString::fromStdString("Unsuspending: " + str_unsuspending +"s")); 
  std::stringstream stream14;
  stream14 << std::fixed << std::setprecision(2) << msg->t_resetting_state;
  std::string str_resetting = stream14.str();
  ui_->resetting_state->setText(QString::fromStdString("Resetting: " + str_resetting +"s")); 
  std::stringstream stream15;
  stream15 << std::fixed << std::setprecision(2) << msg->t_clearing_state;
  std::string str_clearing = stream15.str();
  ui_->clearing_state->setText(QString::fromStdString("Clearing: " + str_clearing +"s")); 
  std::stringstream stream16;
  stream16 << std::fixed << std::setprecision(2) << msg->t_stopping_state;
  std::string str_stopping = stream16.str();
  ui_->stopping_state->setText(QString::fromStdString("Stopping: " + str_stopping +"s"));
}

