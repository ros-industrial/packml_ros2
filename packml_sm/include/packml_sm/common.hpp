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


#ifndef PACKML_SM__COMMON_HPP_
#define PACKML_SM__COMMON_HPP_

namespace packml_sm
{


/* This magic function allows iostream (i.e. ROS_##_STREAM) macros to print out
* enumerations
* see: http://stackoverflow.com/questions/11421432/how-can-i-output-the-value-of-an-enum-class-in-c11
*/


template<typename T>
std::ostream & operator<<(
  typename std::enable_if<std::is_enum<T>::value,
  std::ostream>::type & stream, const T & e)
{
  return stream << static_cast<typename std::underlying_type<T>::type>(e);
}

enum class StatesEnum
{
  UNDEFINED = 0,
  OFF = 1,
  STOPPED = 2,
  STARTING = 3,
  IDLE = 4,
  SUSPENDED = 5,
  EXECUTE = 6,
  STOPPING = 7,
  ABORTING = 8,
  ABORTED = 9,
  HOLDING = 10,
  HELD = 11,
  RESETTING = 100,
  SUSPENDING = 101,
  UNSUSPENDING = 102,
  CLEARING = 103,
  UNHOLDING = 104,
  COMPLETING = 105,
  COMPLETE = 106,


  /* Super states that encapsulate multiple substates with a common transition
  * Not explicitly used in the standard but helpful for consutrcting the state
  * machine.
  */


  ABORTABLE = 200,
  STOPPABLE = 201
};

enum class ModeEnum
{
  UNDEFINED = 0,
  AUTOMATIC = 1,
  SEMI_AUTOMATIC = 2,
  MANUAL = 3,
  IDLE = 4,
  SETUP = 11
};

enum class CmdEnum
{
  UNDEFINED = 0,
  CLEAR = 1,
  START = 2,
  STOP = 3,
  HOLD = 4,
  ABORT = 5,
  RESET = 6,
  ESTOP = 7,
  SUSPEND = 100,
  UNSUSPEND = 101,
  UNHOLD = 102
};
}  // namespace packml_sm
#endif  // PACKML_SM__COMMON_HPP_
