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

#include <QtConcurrent/QtConcurrent>

#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include "packml_sm/state.hpp"
#include "packml_sm/events.hpp"

namespace packml_sm
{

void PackmlState::onEntry(QEvent * /*e*/)  // NOLINT(readability/casting)
{
  std::cout << "Entering state: " << name_.toUtf8().constData() << "(" << state_ << ")" <<
    std::endl;
  emit stateEntered(static_cast<int>(state_), name_);
  enter_time_ = std::chrono::system_clock::now();
}


void PackmlState::onExit(QEvent * /*e*/)  // NOLINT(readability/casting)
{
  std::cout << "Exiting state: " << name_.toUtf8().constData() << "(" << state_ << ")" <<
    std::endl;
  exit_time_ = std::chrono::system_clock::now();
  cummulative_time_ = cummulative_time_ + (exit_time_ - enter_time_);
  std::cout << "Updating cummulative time, for state: " << name_.toUtf8().constData() << "(" <<
    state_ << ") to: " << cummulative_time_.count() << std::endl;
}

void ActingState::onEntry(QEvent * e)
{
  PackmlState::onEntry(e);
  printf("Starting thread for state operation\n");
  function_state_ = QtConcurrent::run(std::bind(&ActingState::operation, this));
}

void ActingState::onExit(QEvent * e)
{
  if (function_state_.isRunning()) {
    printf(
      "State exit triggered early, waiting for state operation to complete\n");
  }
  function_state_.waitForFinished();
  PackmlState::onExit(e);
}


void ActingState::operation()
{
  QEvent * sc;
  if (function_) {
    printf("Executing operational function in acting state\n");
    int error_code = function_();
    if (0 == error_code) {
      sc = new StateCompleteEvent();
    } else {
      std::cout << "Operational function returned error code: " << error_code <<
        std::endl;
      sc = new ErrorEvent(error_code);
    }
  } else {
    std::cout << "Default operation, delaying " << delay_ms << " ms" <<
      std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(delay_ms / 1000.0)));
    printf("Operation delay complete\n");
    sc = new StateCompleteEvent();
  }
  machine()->postEvent(sc);
}

}  // namespace packml_sm
