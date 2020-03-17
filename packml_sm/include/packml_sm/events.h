/**
 * @license Software License Agreement (Apache License)
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


#ifndef PACKML_SM__EVENTS_H_
#define PACKML_SM__EVENTS_H_

#include "QEvent"
#include "QAbstractTransition"
#include "packml_sm/common.h"
#include "packml_sm/state.h"

namespace packml_sm
{
static int PACKML_CMD_EVENT_TYPE = QEvent::User + 1;
static int PACKML_STATE_COMPLETE_EVENT_TYPE = QEvent::User + 2;
static int PACKML_ERROR_EVENT_TYPE = QEvent::User + 3;

struct CmdEvent : public QEvent
{
  static CmdEvent * clear()
  {
    return new CmdEvent(CmdEnum::CLEAR);
  }
  static CmdEvent * start()
  {
    return new CmdEvent(CmdEnum::START);
  }
  static CmdEvent * stop()
  {
    return new CmdEvent(CmdEnum::STOP);
  }
  static CmdEvent * hold()
  {
    return new CmdEvent(CmdEnum::HOLD);
  }
  static CmdEvent * abort()
  {
    return new CmdEvent(CmdEnum::ABORT);
  }
  static CmdEvent * reset()
  {
    return new CmdEvent(CmdEnum::RESET);
  }
  static CmdEvent * estop()
  {
    return new CmdEvent(CmdEnum::ESTOP);
  }
  static CmdEvent * suspend()
  {
    return new CmdEvent(CmdEnum::SUSPEND);
  }
  static CmdEvent * unsuspend()
  {
    return new CmdEvent(CmdEnum::UNSUSPEND);
  }
  static CmdEvent * unhold()
  {
    return new CmdEvent(CmdEnum::UNHOLD);
  }

  explicit  CmdEvent(const CmdEnum &cmd_value)
  : QEvent(QEvent::Type(PACKML_CMD_EVENT_TYPE)),
    cmd(cmd_value) {}

  CmdEnum cmd;
};

struct StateCompleteEvent : public QEvent
{
  StateCompleteEvent()
  : QEvent(QEvent::Type(PACKML_STATE_COMPLETE_EVENT_TYPE)) {}
};

struct ErrorEvent : public QEvent
{
  explicit  ErrorEvent(const int &code_value)
  : QEvent(QEvent::Type(PACKML_ERROR_EVENT_TYPE)),
    code(code_value),
    name(),
    description() {}

  ErrorEvent(const int &code_value, const QString &name_value,
    const QString &description_value)
  : QEvent(QEvent::Type(PACKML_ERROR_EVENT_TYPE)),
    code(code_value),
    name(name_value),
    description(description_value) {}

  int code;
  QString name;
  QString description;
};


}  // namespace packml_sm

#endif  // PACKML_SM__EVENTS_H_
