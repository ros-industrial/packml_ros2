// Copyright (c) 2016 Shaun Edwards
// Copyright (c) 2019 Dejanira Araiza Illan, ROS-Industrial Asia Pacific
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

#include <QCoreApplication>
#include <QTimer>
#include <gtest/gtest.h>
#include <thread>
#include <iostream>
#include <chrono>
#include <memory>
#include "packml_sm/common.hpp"
#include "packml_sm/events.hpp"
#include "packml_sm/state_machine.hpp"
#include "rclcpp/rclcpp.hpp"

void qtWorker(int argc, char * argv[])
{
  printf("Here\n");
  QCoreApplication a(argc, argv);
  a.exec();
  printf("There\n");
}

/**
 * @brief waitForState - returns true if the current state of the state machine (sm) matches the queried state
 * @param state - state enumeration to wait for
 * @param sm - top level state machine
 * @return true if state changes to queried state before internal timeout
 */


bool waitForState(packml_sm::StatesEnum state, packml_sm::StateMachine & sm)
{
  const double TIMEOUT = 2.0;
  const int SAMPLES = 50;
  rclcpp::Rate r(static_cast<double>(SAMPLES) / TIMEOUT);
  for (int ii = 0; ii < SAMPLES; ++ii) {
    if (sm.getCurrentState() == static_cast<int>(state)) {
      std::cout << "State changed to " << state << std::endl;
      return true;
    }
    std::cout << "Waiting for state to change to " << state << std::endl;
    r.sleep();
  }
  return false;
}

int success()
{
  printf("Beginning success() method\n");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  printf("Execute method complete\n");
  return 0;
}

int fail()
{
  printf("Beginning fail method\n");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  printf("Execute method complete\n");
  return -1;
}

TEST(Packml_sm, single_cycle_state_machine_set_execute_state)
{
  std::shared_ptr<packml_sm::StateMachine> sm = packml_sm::StateMachine::singleCyleSM();
  sm->setExecute(std::bind(success));
  sm->activate();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  bool answer1 = sm->clear();
  ASSERT_TRUE(answer1 == true);
  bool answer2 = waitForState(packml_sm::StatesEnum::STOPPED, *sm);
  ASSERT_TRUE(answer2 == true);
  bool answer3 = sm->reset();
  ASSERT_TRUE(answer3 == true);
  bool answer4 = waitForState(packml_sm::StatesEnum::IDLE, *sm);
  ASSERT_TRUE(answer4 == true);
  bool answer5 = sm->start();
  ASSERT_TRUE(answer5 == true);
  bool answer6 = waitForState(packml_sm::StatesEnum::COMPLETE, *sm);
  ASSERT_TRUE(answer6 == true);
  bool answer7 = sm->reset();
  ASSERT_TRUE(answer7 == true);
  bool answer8 = waitForState(packml_sm::StatesEnum::IDLE, *sm);
  ASSERT_TRUE(answer8 == true);
  sm->setExecute(std::bind(fail));
  bool answer9 = sm->start();
  ASSERT_TRUE(answer9 == true);
  bool answer10 = waitForState(packml_sm::StatesEnum::ABORTED, *sm);
  ASSERT_TRUE(answer10 == true);
}

TEST(Packml_sm, single_Cycle_state_machine_follow_diagram)
{
  packml_sm::SingleCycle sm;
  EXPECT_FALSE(sm.isActive());
  sm.setExecute(std::bind(success));
  sm.activate();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  bool answer1 = sm.isActive();
  ASSERT_TRUE(answer1 == true);
  bool answer2 = waitForState(packml_sm::StatesEnum::ABORTED, sm);
  ASSERT_TRUE(answer2 == true);
  bool answer3 = sm.isActive();
  ASSERT_TRUE(answer3 == true);
  sm.clear();
  bool answer6 = waitForState(packml_sm::StatesEnum::STOPPED, sm);
  ASSERT_TRUE(answer6 == true);
  bool answer7 = sm.reset();
  ASSERT_TRUE(answer7 == true);
  bool answer9 = waitForState(packml_sm::StatesEnum::IDLE, sm);
  ASSERT_TRUE(answer9 == true);
  bool answer10 = sm.start();
  ASSERT_TRUE(answer10 == true);
  bool answer12 = waitForState(packml_sm::StatesEnum::EXECUTE, sm);
  ASSERT_TRUE(answer12 == true);
  bool answer14 = waitForState(packml_sm::StatesEnum::COMPLETE, sm);
  ASSERT_TRUE(answer14 == true);
  bool answer15 = sm.reset();
  ASSERT_TRUE(answer15 == true);
  bool answer17 = waitForState(packml_sm::StatesEnum::IDLE, sm);
  ASSERT_TRUE(answer17 == true);
  bool answer18 = sm.start();
  ASSERT_TRUE(answer18 == true);
  bool answer20 = waitForState(packml_sm::StatesEnum::EXECUTE, sm);
  ASSERT_TRUE(answer20 == true);
  bool answer21 = sm.hold();
  ASSERT_TRUE(answer21 == true);
  bool answer23 = waitForState(packml_sm::StatesEnum::HELD, sm);
  ASSERT_TRUE(answer23 == true);
  bool answer24 = sm.unhold();
  ASSERT_TRUE(answer24 == true);
  bool answer26 = waitForState(packml_sm::StatesEnum::EXECUTE, sm);
  ASSERT_TRUE(answer26 == true);
  bool answer27 = sm.suspend();
  ASSERT_TRUE(answer27 == true);
  bool answer29 = waitForState(packml_sm::StatesEnum::SUSPENDED, sm);
  ASSERT_TRUE(answer29 == true);
  bool answer30 = sm.unsuspend();
  ASSERT_TRUE(answer30 == true);
  bool answer32 = waitForState(packml_sm::StatesEnum::EXECUTE, sm);
  ASSERT_TRUE(answer32 == true);
  bool answer33 = sm.stop();
  ASSERT_TRUE(answer33 == true);
  bool answer35 = waitForState(packml_sm::StatesEnum::STOPPED, sm);
  ASSERT_TRUE(answer35 == true);
  bool answer36 = sm.abort();
  ASSERT_TRUE(answer36 == true);
  bool answer38 = waitForState(packml_sm::StatesEnum::ABORTED, sm);
  ASSERT_TRUE(answer38 == true);
  sm.deactivate();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  EXPECT_FALSE(sm.isActive());
}

TEST(Packml_sm, continuous_execution_state_machine_follow_diagram)
{
  packml_sm::ContinuousCycle sm;
  EXPECT_FALSE(sm.isActive());
  sm.setExecute(std::bind(success));
  sm.activate();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  EXPECT_TRUE(sm.isActive());
  bool answer1 = waitForState(packml_sm::StatesEnum::ABORTED, sm);
  ASSERT_TRUE(answer1 == true);
  bool answer2 = sm.isActive();
  ASSERT_TRUE(answer2 == true);
  bool answer3 = sm.clear();
  ASSERT_TRUE(answer3 == true);
  bool answer5 = waitForState(packml_sm::StatesEnum::STOPPED, sm);
  ASSERT_TRUE(answer5 == true);
  bool answer6 = sm.reset();
  ASSERT_TRUE(answer6 == true);
  bool answer8 = waitForState(packml_sm::StatesEnum::IDLE, sm);
  ASSERT_TRUE(answer8 == true);
  bool answer9 = sm.start();
  ASSERT_TRUE(answer9 == true);
  bool answer11 = waitForState(packml_sm::StatesEnum::EXECUTE, sm);
  ASSERT_TRUE(answer11 == true);
  bool answer13 = waitForState(packml_sm::StatesEnum::COMPLETE, sm);
  ASSERT_TRUE(answer13 == false);
  bool answer14 = sm.hold();
  ASSERT_TRUE(answer14 == true);
  bool answer16 = waitForState(packml_sm::StatesEnum::HELD, sm);
  ASSERT_TRUE(answer16 == true);
  bool answer17 = sm.unhold();
  ASSERT_TRUE(answer17 == true);
  bool answer19 = waitForState(packml_sm::StatesEnum::EXECUTE, sm);
  ASSERT_TRUE(answer19 == true);
  bool answer20 = sm.suspend();
  ASSERT_TRUE(answer20 == true);
  bool answer22 = waitForState(packml_sm::StatesEnum::SUSPENDED, sm);
  ASSERT_TRUE(answer22 == true);
  bool answer23 = sm.unsuspend();
  ASSERT_TRUE(answer23 == true);
  bool answer25 = waitForState(packml_sm::StatesEnum::EXECUTE, sm);
  ASSERT_TRUE(answer25 == true);
  bool answer26 = sm.stop();
  ASSERT_TRUE(answer26 == true);
  bool answer28 = waitForState(packml_sm::StatesEnum::STOPPED, sm);
  ASSERT_TRUE(answer28 == true);
  bool answer29 = sm.abort();
  ASSERT_TRUE(answer29 == true);
  bool answer31 = waitForState(packml_sm::StatesEnum::ABORTED, sm);
  ASSERT_TRUE(answer31 == true);
  sm.deactivate();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  EXPECT_FALSE(sm.isActive());
}

TEST(Packml_sm, testing_failed_state_transition_executions)
{
  packml_sm::SingleCycle sm;
  sm.setExecute(std::bind(success));
  sm.activate();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  waitForState(packml_sm::StatesEnum::ABORTED, sm);
  bool answer3 = sm.start();
  ASSERT_TRUE(answer3 == false);
  bool answer4 = sm.reset();
  ASSERT_TRUE(answer4 == false);
  bool answer5 = sm.hold();
  ASSERT_TRUE(answer5 == false);
  bool answer6 = sm.unhold();
  ASSERT_TRUE(answer6 == false);
  bool answer7 = sm.suspend();
  ASSERT_TRUE(answer7 == false);
  bool answer8 = sm.unsuspend();
  ASSERT_TRUE(answer8 == false);
  bool answer9 = sm.stop();
  ASSERT_TRUE(answer9 == false);
  bool answer10 = sm.abort();
  ASSERT_TRUE(answer10 == false);
  sm.clear();
  waitForState(packml_sm::StatesEnum::STOPPED, sm);
  bool answer12 = sm.unhold();
  ASSERT_TRUE(answer12 == false);
  bool answer13 = sm.clear();
  ASSERT_TRUE(answer13 == false);
}

TEST(Packml_sm, testing_init_function)
{
  char ** names;
  names = new char *[10];
  packml_sm::init(2, names);
  delete[] names;
}

TEST(Packml_sm, testing_setResetting)
{
  packml_sm::SingleCycle sm;
  sm.setResetting(std::bind(success));
  sm.activate();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  waitForState(packml_sm::StatesEnum::ABORTED, sm);
  EXPECT_TRUE(sm.isActive());
}

int main(int argc, char ** argv)
{
  std::thread thr(qtWorker, argc, argv);
  // qtWorker(argc, argv);
  while (NULL == QCoreApplication::instance()) {
    printf("Waiting for QCore application to start\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  thr.detach();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
