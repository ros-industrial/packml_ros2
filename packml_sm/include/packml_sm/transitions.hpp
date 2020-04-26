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


#ifndef PACKML_SM__TRANSITIONS_HPP_
#define PACKML_SM__TRANSITIONS_HPP_

#include <QtGui>
#include "QEvent"
#include "QAbstractTransition"
#include "packml_sm/common.hpp"
#include "packml_sm/state.hpp"
#include "rclcpp/rclcpp.hpp"


/**
* @brief Class that implements transitions between the states of a standard PackML state machine
*/
namespace packml_sm
{
class CmdTransition : public QAbstractTransition
{
public:
  /**
  * @brief Function to transition to the clear state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * clear(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::CLEAR, "clear", from, to);
  }


  /**
  * @brief Function to transition to the start state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * start(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::START, "start", from, to);
  }


  /**
  * @brief Function to transition to the stop state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * stop(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::STOP, "stop", from, to);
  }


  /**
  * @brief Function to transition to the hold state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * hold(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::HOLD, "hold", from, to);
  }


  /**
  * @brief Function to transition to the abort state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * abort(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::ABORT, "abort", from, to);
  }


  /**
  * @brief Function to transition to the reset state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * reset(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::RESET, "reset", from, to);
  }


  /**
  * @brief Function to transition to the e stop state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * estop(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::ESTOP, "estop", from, to);
  }


  /**
  * @brief Function to transition to the abort state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * suspend(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::SUSPEND, "abort", from, to);
  }


  /**
  * @brief Function to transition to the unsuspend state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * unsuspend(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::UNSUSPEND, "unsuspend", from, to);
  }


  /**
  * @brief Function to transition to the unhold state
  * @param from - original state
  * @param to - ending state
  */
  static CmdTransition * unhold(PackmlState & from, PackmlState & to)
  {
    return new CmdTransition(CmdEnum::UNHOLD, "unhold", from, to);
  }


  /**
  * @brief Function to define a transition from a triggering event
  * @param cmd_value - number of the new requested state
  * @param name_value - name of the new requested state
  */
  CmdTransition(const CmdEnum & cmd_value, const QString & name_value)
  : cmd(cmd_value), name(name_value) {}


  /**
  * @brief Function to define a transition from a triggering event
  * @param cmd_value - number of the new requested state
  * @param name_value - name of the new requested state
  * @param from - original state
  * @param to - ending state
  */
  CmdTransition(
    const CmdEnum & cmd_value, const QString & name_value,
    PackmlState & from, PackmlState & to);

protected:
  /**
  * @brief Function to check if a transition is valid
  * @param e - triggering event
  */
  virtual bool eventTest(QEvent * e);


  /**
  * @brief Function to trigger an action when the transition is happening
  * @param e - triggering event
  */
  virtual void onTransition(QEvent * e) {std::cout << e << std::endl;}


  /**
  * @brief Number of the state
  */
  CmdEnum cmd;


  /**
  * @brief Name of the state
  */
  QString name;
};


/**
* @brief Class defining a transition that has been completed
*/
class StateCompleteTransition : public QAbstractTransition
{
public:
  /**
  * @brief Constructor of the class
  */
  StateCompleteTransition() {}


  /**
  * @brief Constructor of the class
  * @param from - original state
  * @param to - ending state
  */
  StateCompleteTransition(PackmlState & from, PackmlState & to);


  /**
  * @brief Destructor of the class
  */
  virtual ~StateCompleteTransition() {}

protected:
  /**
  * @brief Function to check if the transition is valid
  * @param e - triggering event
  */
  virtual bool eventTest(QEvent * e);


  /**
  * @brief Function to trigger an action when the transition is happening
  * @param e - triggering event
  */
  virtual void onTransition(QEvent * e) {std::cout << e << std::endl;}

private:
};


/**
* @brief Class to define transitions that are not valid
*/
class ErrorTransition : public QAbstractTransition
{
public:
  /**
  * @brief Constructor of the class
  */
  ErrorTransition() {}


  /**
  * @brief Constructor of the class
  * @param from - original state
  * @param to - ending desired state
  */
  ErrorTransition(PackmlState & from, PackmlState & to);


  /**
  * @brief Destructor of the class
  */
  virtual ~ErrorTransition() {}

protected:
  /**
  * @brief Function to check if the transition is valid
  * @param e - triggering event
  */
  virtual bool eventTest(QEvent * e);


  /**
  * @brief Function to trigger an action when the transition is happening
  * @param e - triggering event
  */
  virtual void onTransition(QEvent * e) {std::cout << e << std::endl;}
};

}  // namespace packml_sm
#endif  // PACKML_SM__TRANSITIONS_HPP_
