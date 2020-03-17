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


#include <pluginlib/class_list_macros.hpp>  // Plugin generation
#include "packml_plugin/packml_plugin.h" 
#include "packml_msgs/srv/all_status.hpp"

PackmlPlugin::PackmlPlugin(QWidget* parent) : rviz_common::Panel(parent)
{
  // Brings up the GUI
  widget_ = new PackmlWidget();
  // Panel setup in RVIZ
  layout = new QVBoxLayout(this);
  layout->addWidget(widget_);
  setLayout(layout);
  printf("Loaded Packml Navigation panel\n");
}
PLUGINLIB_EXPORT_CLASS(PackmlPlugin, rviz_common::Panel)
