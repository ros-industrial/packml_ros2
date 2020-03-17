#!/usr/bin/env python
# 
# Software License Agreement
# Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific 
# Advanced Remanufacturing and Technology Centre
# A*STAR Research Entities (Co. Registration No. 199702110H)
#
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# Contributors: Dejanira Araiza Illan, Derrick Ang Ming Yan
#


from opcua import Client, ua
from opcua.ua import ua_binary as uabin
from opcua.common.methods import call_method
from opcua import common
from packml_msgs.srv import Transition

import time
import rclpy 
from rclpy.node import Node

class DriverSender(Node):
  """
  This class creates a ROS2 node and a service server to control the transitions of the PLC's state 
  machine upon pressing buttons on the GUI. If the GUI requests a transition service, the PLC is 
  sent a corresponding signal. 
  """
  def __init__(self):
    super().__init__('driver_sender')
    self.srv = self.create_service(Transition,'transition',self.transRequest)
    self.client = Client("opc.tcp://192.168.125.2:4840/freeopcua/server/")
  def connect(self):
    self.client.connect()
  def __exit__(self, exc_type, exc_val, exc_tb):
    self.client.disconnect()
  def transRequest(self, req, res):
    command_rtn = False
    command_valid = True
    command_int = req.command
    print("Evaluating transition request command: " + str(command_int))
    try:
      if command_int == 5:
        cmdAbort = self.client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Abort\"")
        cmdAbort.set_attribute(ua.AttributeIds.Value,ua.DataValue(True))
        time.sleep(0.1)
        print("Sending Abort Command . . .")
        command_rtn = True
      elif command_int == 7:
        cmdStop = self.client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Stop\"")
        cmdStop.set_attribute(ua.AttributeIds.Value,ua.DataValue(True))
        time.sleep(0.1)		
        print("Sending Stop Command . . .")
        command_rtn = True
      elif command_int == 1:
        cmdClear = self.client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Clear\"")
        cmdClear.set_attribute(ua.AttributeIds.Value,ua.DataValue(True))
        time.sleep(0.1)		
        print("Sending Clear Command . . .")
        command_rtn = True
      elif command_int == 4:
        cmdHold = self.client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Hold\"")
        cmdHold.set_attribute(ua.AttributeIds.Value,ua.DataValue(True))
        time.sleep(0.1)		
        print("Sending Hold Command . . .")
        command_rtn = True
      elif command_int == 6:
        cmdReset = self.client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Reset\"")
        cmdReset.set_attribute(ua.AttributeIds.Value,ua.DataValue(True))
        time.sleep(0.1)
        print("Sending Reset Command . . .")
        command_rtn = True
      elif command_int == 2:
        cmdStart = self.client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Start\"")
        cmdStart.set_attribute(ua.AttributeIds.Value,ua.DataValue(True))
        time.sleep(0.1)
        print("Sending Start Command . . .")
        command_rtn = True
      elif command_int ==  3:
        cmdStop = self.client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Stop\"")
        cmdStop.set_attribute(ua.AttributeIds.Value,ua.DataValue(True))
        time.sleep(0.05)		
        print("Sending Stop Command . . .")
        command_rtn = True
      elif command_int == 100:
        cmdSuspend = self.client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Suspend\"")
        cmdSuspend.set_attribute(ua.AttributeIds.Value,ua.DataValue(True))
        time.sleep(0.1)		
        print("Sending Suspend Command . . .")
        command_rtn = True
      elif command_int ==  102:
        cmdUnhold = self.client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Unhold\"")
        cmdUnhold.set_attribute(ua.AttributeIds.Value,ua.DataValue(True))
        time.sleep(0.1)		
        print("Sending Unhold Command . . .")
        command_rtn = True
      elif command_int ==  101:
        cmdUnsuspend = self.client.get_node("ns=3;s=\"PackML_Status\".\"EM00\".\"Unit\".\"Cmd_Unsuspend\"")
        cmdUnsuspend.set_attribute(ua.AttributeIds.Value,ua.DataValue(True))
        time.sleep(0.1)		
        print("Sending Unsuspend Command . . .")
        command_rtn = True
      else:
        command_valid = False
    except KeyboardInterrupt:
      pass
    if command_valid==True:
      if command_rtn==True:
        print("Successful transition request command: " + str(command_int))
        res.success = True
        res.error_code = res.SUCCESS
      else:
        res.success = False
        res.error_code = res.INVALID_TRANSITION_REQUEST
    else:
      res.success = False
      res.error_code = res.UNRECGONIZED_REQUEST
    return res

def main(args=None):
  rclpy.init(args=args)
  driver_sender=DriverSender()
  driver_sender.connect()
  rclpy.spin(driver_sender)

if __name__ == '__main__':
	 main()
