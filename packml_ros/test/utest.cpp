/**
 * @license Apache 2.0
 *
 * @copyright Copyright (c) 2016 Shaun Edwards
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "packml_ros/packml_ros.h"
#include <chrono>
#include "rclcpp/rclcpp.hpp"
using ::testing::Return;

class MockSMNode : public SMNode {
public:
  MockSMNode(rclcpp::Node::SharedPtr node) : SMNode(node) {};
  MOCK_METHOD0(getCurrentState, int());

};


TEST(Packml_ros, constructor_test)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  SMNode thenode(node);
  rclcpp::spin_some(node);
}


TEST(Packml_ros, test_transRequest_commands)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto transition_client_ = node->create_client<packml_msgs::srv::Transition>("transition"); 
  SMNode thenode(node);
  auto trans = std::make_shared<packml_msgs::srv::Transition::Request>();
  trans->command = 1;
  auto result_future = transition_client_->async_send_request(trans);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 6;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 2;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 100;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 101;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 4;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 102;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 7;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 5;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 1;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  trans->command = 6;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  trans->command = 3;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == true)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 100;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == false)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  trans->command = 105;
  result_future = transition_client_->async_send_request(trans);
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success == false)
      check1 = true;
  }
  ASSERT_TRUE(check1 == true);
  std::this_thread::sleep_for(std::chrono::seconds(10));
}

TEST(Packml_ros, test_statusRequest_commands_2)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(2));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_3)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(3));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_4)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(4));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_5)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(5));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_6)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(6));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_7)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(7));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_8)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(8));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_9)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(9));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_10)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(10));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_11)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(11));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_100)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(100));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_101)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(101));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_102)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(102));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_103)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(103));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_104)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(104));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_105)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(105));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_106)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(106));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_statusRequest_commands_wrong)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  auto status_client_ = node->create_client<packml_msgs::srv::AllStatus>("allStatus");
  MockSMNode thenode(node);
  EXPECT_CALL(thenode, getCurrentState()).WillOnce(Return(0));
  auto update = std::make_shared<packml_msgs::srv::AllStatus::Request>();
  update->command = true;
  auto result_future = status_client_->async_send_request(update);
  bool check1 = false;
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::executor::FutureReturnCode::SUCCESS) {
      check1 = true;
  } else {
    check1 = false;
  }
  EXPECT_TRUE(check1 == true);
}

TEST(Packml_ros, test_execute_method)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  SMNode thenode(node);
  int result = thenode.myExecuteMethod();
  EXPECT_EQ(result,0);
}

TEST(Packml_ros, test_getcurrentstate_method)
{
  auto node = rclcpp::Node::make_shared("packml_ros_node");
  SMNode thenode(node);
  int result = thenode.getCurrentState();
  EXPECT_NE(result,0);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::thread thr(qtWorker, argc, argv);
  // qtWorker(argc, argv);
  while(NULL == QCoreApplication::instance()){
    printf("Waiting for QCore application to start\n");
  }
  thr.detach();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}






