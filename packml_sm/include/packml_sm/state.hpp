// Copyright (c) 2016 Shaun Edwards
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


#ifndef PACKML_SM__STATE_HPP_
#define PACKML_SM__STATE_HPP_


#include <QtGui>

#include <chrono>
#include <functional>
#include "QState"
#include "QEvent"
#include "QAbstractTransition"
#include "rclcpp/rclcpp.hpp"
#include "packml_sm/common.hpp"

namespace packml_sm
{

struct PackmlState : public QState
{
  Q_OBJECT

public:
  PackmlState(StatesEnum state_value, QString name_value)
  : state_(state_value),
    name_(name_value),
    cummulative_time_(0) {}

  PackmlState(StatesEnum state_value, QString name_value, QState * super_state)
  : QState(super_state),
    state_(state_value),
    name_(name_value),
    cummulative_time_(0) {}

  StatesEnum state() const {return state_;}
  const QString name() const {return name_;}
  virtual ~PackmlState() {}

signals:
  void stateEntered(int value, QString name);

protected:
  StatesEnum state_;
  QString name_;
  std::chrono::time_point<std::chrono::system_clock> enter_time_;
  std::chrono::time_point<std::chrono::system_clock> exit_time_;
  std::chrono::duration<double> cummulative_time_;
  virtual void onEntry(QEvent * e);
  virtual void operation() {}
  virtual void onExit(QEvent * e);
};

struct WaitState : public PackmlState
{
public:
  static WaitState * Abortable()
  {
    return new WaitState(StatesEnum::ABORTABLE, CmdEnum::ABORT, "Abortable");
  }
  static WaitState * Stoppable(QState * abortable)
  {
    return new WaitState(StatesEnum::STOPPABLE, CmdEnum::ABORT, "Stoppable", abortable);
  }
  static WaitState * Idle(QState * stoppable)
  {
    return new WaitState(StatesEnum::IDLE, CmdEnum::START, "Idle", stoppable);
  }
  static WaitState * Held(QState * stoppable)
  {
    return new WaitState(StatesEnum::HELD, CmdEnum::UNHOLD, "Held", stoppable);
  }
  static WaitState * Complete(QState * stoppable)
  {
    return new WaitState(StatesEnum::COMPLETE, CmdEnum::RESET, "Complete", stoppable);
  }
  static WaitState * Suspended(QState * stoppable)
  {
    return new WaitState(StatesEnum::SUSPENDED, CmdEnum::UNSUSPEND, "Suspended", stoppable);
  }
  static WaitState * Stopped(QState * abortable)
  {
    return new WaitState(StatesEnum::STOPPED, CmdEnum::RESET, "Stopped", abortable);
  }
  static WaitState * Aborted()
  {
    return new WaitState(StatesEnum::ABORTED, CmdEnum::CLEAR, "Aborted");
  }
  WaitState(StatesEnum state_value, CmdEnum exit_cmd_value, QString name_value)
  : PackmlState(state_value, name_value),
    exit_cmd(exit_cmd_value) {}
  WaitState(
    StatesEnum state_value, CmdEnum exit_cmd_value, QString name_value,
    QState * super_state)
  : PackmlState(state_value, name_value, super_state),
    exit_cmd(exit_cmd_value) {}
  virtual ~WaitState() {}

private:
  CmdEnum exit_cmd;
};

struct ActingState : public PackmlState
{
public:
  static ActingState * Resetting(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::RESETTING, "Resetting", stoppable, delay_ms_value);
  }
  static ActingState * Starting(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::STARTING, "Starting", stoppable, delay_ms_value);
  }
  static ActingState * Unholding(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::UNHOLDING, "Un-Holding", stoppable, delay_ms_value);
  }
  static ActingState * Unsuspending(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::UNSUSPENDING, "Un-Suspending", stoppable, delay_ms_value);
  }
  static ActingState * Holding(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::HOLDING, "Holding", stoppable, delay_ms_value);
  }
  static ActingState * Suspending(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::SUSPENDING, "Suspending", stoppable, delay_ms_value);
  }
  static ActingState * Execute(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::EXECUTE, "Execute", stoppable, delay_ms_value);
  }
  static ActingState * Execute(QState * stoppable, std::function<int()> function_value)
  {
    return new ActingState(StatesEnum::EXECUTE, "Execute", stoppable, function_value);
  }
  static ActingState * Completing(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::COMPLETING, "Completing", stoppable, delay_ms_value);
  }
  static ActingState * Aborting(int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::ABORTING, "Aborting", delay_ms_value);
  }
  static ActingState * Clearing(QState * abortable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::CLEARING, "Clearing", abortable, delay_ms_value);
  }
  static ActingState * Stopping(QState * abortable, int delay_ms_value = 200)
  {
    return new ActingState(StatesEnum::STOPPING, "Aborting", abortable, delay_ms_value);
  }
  ActingState(StatesEnum state_value, const char * name_value, int delay_ms_value = 200)
  : PackmlState(state_value, QString(name_value)),
    delay_ms(delay_ms_value) {}
  ActingState(
    StatesEnum state_value, const QString & name_value, QState * super_state,
    int delay_ms_value = 200)
  : PackmlState(state_value, name_value, super_state),
    delay_ms(delay_ms_value) {}
  ActingState(
    StatesEnum state_value, const char * name_value, QState * super_state,
    std::function<int()> function_value)
  : PackmlState(state_value, QString(name_value), super_state),
    function_(function_value) {}
  bool setOperationMethod(std::function<int()> function_value)
  {
    function_ = function_value;
    return true;
  }
  virtual void operation();
  virtual ~ActingState() {}

protected:
  virtual void onEntry(QEvent * e);
  virtual void onExit(QEvent * e);

private:
  int delay_ms;
  std::function<int()> function_;
  QFuture<void> function_state_;
};

typedef ActingState DualState;
}  // namespace packml_sm

#endif  // PACKML_SM__STATE_HPP_
