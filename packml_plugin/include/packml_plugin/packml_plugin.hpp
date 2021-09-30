// Software License Agreement (Apache License)
//
// Copyright (c) 2017 Austin Deric
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

#ifndef PACKML_PLUGIN__PACKML_PLUGIN_HPP_
#define PACKML_PLUGIN__PACKML_PLUGIN_HPP_

#include <QtGui>
#include <QtCore>
#include <QtWidgets>
#include <rviz_common/panel.hpp>
#include "packml_widget.hpp"


/**
* @brief This class creates the plugin for RViz with a Widget (GUI) to control and monitor
* a PackML state machine
*/
class PackmlPlugin : public rviz_common::Panel
{
  Q_OBJECT

public:
  /**
  * @brief Constructor of the class
  */
  explicit PackmlPlugin(QWidget * parent = 0);


  /**
  * @brief Destructor of the class
  */
  ~PackmlPlugin() override = default;

protected:
  /**
  * @brief Pointer to the GUI layout
  */
  QVBoxLayout * layout;


  /**
  * @brief Pointer to the Widget object
  */
  PackmlWidget * widget_;
};

#endif  // PACKML_PLUGIN__PACKML_PLUGIN_HPP_
