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
from packml_msgs.srv import AllStatus
import time
import rclpy 
import threading
from rclpy.node import Node

newvals = [False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False]
time_stopped = 0.0
time_idle = 0.0
time_starting = 0.0
time_execute = 0.0
time_completing = 0.0
time_complete = 0.0
time_clearing = 0.0
time_suspended = 0.0
time_aborting = 0.0
time_aborted = 0.0
time_holding = 0.0
time_held = 0.0
time_unholding = 0.0
time_suspending = 0.0
time_unsuspending = 0.0
time_resetting = 0.0
time_stopping = 0.0
thee = threading.Event()

class HelloClient:
  """
  This class creates an OPCUA client to connect to the PLC once it has been configured on its end.
  """
  def __init__(self, endpoint):
    self.client = Client(endpoint)
  def __enter__(self):
    self.client.connect()
    return self.client
  def __exit__(self, exc_type, exc_val, exc_tb):
    self.client.disconnect()

class DriverListener(Node):
  """
  This class creates a ROS 2 node and a service server to update the state machine displayed in the 
  GUI with the current state and the elapsed time in a state for all the execution of the machine. 
  The state of the PLC is queried at a set rate.
  """
  def __init__(self):
    super().__init__('driver_listener')
    self.srv = self.create_service(AllStatus,'allStatus',self.sendData)
  def sendData(self,req,res):
    global time_stopped
    global time_idle
    global time_starting
    global time_execute
    global time_completing
    global time_complete
    global time_clearing
    global time_suspended
    global time_aborting
    global time_aborted
    global time_holding
    global time_held
    global time_unholding
    global time_suspending
    global time_unsuspending
    global time_resetting
    global time_stopping
    global newvals
    res.stopped_state = newvals[0]
    res.idle_state = newvals[1]
    res.starting_state = newvals[2]
    res.execute_state = newvals[3]
    res.completing_state = newvals[4]
    res.complete_state = newvals[5]
    res.clearing_state = newvals[6]
    res.suspended_state = newvals[7]
    res.aborting_state = newvals[8]
    res.aborted_state = newvals[9]
    res.holding_state = newvals[10]
    res.held_state = newvals[11]
    res.unholding_state = newvals[12]
    res.suspending_state = newvals[13]
    res.unsuspending_state = newvals[14]
    res.resetting_state = newvals[15]
    res.stopping_state = newvals[16]
    res.t_stopped_state = time_stopped
    res.t_idle_state = time_idle
    res.t_starting_state = time_starting
    res.t_execute_state = time_execute
    res.t_completing_state = time_completing
    res.t_complete_state = time_complete
    res.t_clearing_state = time_clearing
    res.t_suspended_state = time_suspended
    res.t_aborting_state = time_aborting
    res.t_aborted_state = time_aborted
    res.t_holding_state = time_holding
    res.t_held_state = time_held
    res.t_unholding_state = time_unholding
    res.t_suspending_state = time_suspending
    res.t_unsuspending_state = time_unsuspending
    res.t_resetting_state = time_resetting
    res.t_stopping_state =time_stopping
    return res

def PLCListener(e):
  """
  Function that creates the connection with the PLC, monitors the state in its state machine, 
  and keeps a counter of the elapsed time in the same state. Runs as an independent thread.
  """
  # Resetting the time counters every time the connection between the PLC and ROS is initialized
  global time_stopped
  time_stopped=0.0
  global time_idle
  time_idle=0.0
  global time_starting
  time_starting=0.0
  global time_execute
  time_execute=0.0
  global time_completing
  time_completing=0.0
  global time_complete
  time_complete=0.0
  global time_clearing
  time_clearing=0.0
  global time_suspended
  time_suspended=0.0
  global time_aborting
  time_aborting=0.0
  global time_aborted
  time_aborted=0.0
  global time_holding
  time_holding=0.0
  global time_held
  time_held=0.0
  global time_unholding
  time_unholding=0.0
  global time_suspending
  time_suspending=0.0
  global time_unsuspending
  time_unsuspending=0.0
  global time_resetting
  time_resetting=0.0
  global time_stopping
  time_stopping=0.0
  global_time=time.time() # The connection is active, the time counters start with this value
  # Open a connection with the PLC
  with HelloClient("opc.tcp://192.168.125.2:4840/freeopcua/server/") as client:
    try:
      while not e.isSet():
        stopped_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Stopped\"")
        idle_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Idle\"")
        starting_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Starting\"")
        execute_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Execute\"")
        completing_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Completing\"")
        complete_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Complete\"")
        clearing_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Clearing\"")
        suspended_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Suspended\"")
        aborting_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Aborting\"")	
        aborted_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Aborted\"")	  
        holding_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Holding\"")  
        held_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Held\"")
        unholding_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Unholding\"")   
        suspending_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Suspending\"")
        unsuspending_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Unsuspending\"")
        resetting_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Resetting\"")
        stopping_0 = client.get_node("ns=3;s=\"PackML_Status\".\"Sts\".\"State\".\"Stopping\"")
        """
        print (stopped_0.get_value(), idle_0.get_value(), starting_0.get_value(), 
          execute_0.get_value(), completing_0.get_value(), complete_0.get_value(), 
          clearing_0.get_value(), suspended_0.get_value(), aborting_0.get_value(), 
          aborted_0.get_value(), holding_0.get_value(),held_0.get_value(), unholding_0.get_value(), 
          suspending_0.get_value(),  unsuspending_0.get_value(), resetting_0.get_value(), 
          stopping_0.get_value())
        """
        # Checking and updating the time counters for each state, approximated to 1 decimal place
        global newvals
        newvals = [stopped_0.get_value(), idle_0.get_value(), starting_0.get_value(), 
          execute_0.get_value(), completing_0.get_value(), complete_0.get_value(), 
          clearing_0.get_value(), suspended_0.get_value(), aborting_0.get_value(),  
          aborted_0.get_value(), holding_0.get_value(), held_0.get_value(), unholding_0.get_value(),
          suspending_0.get_value(), unsuspending_0.get_value(), resetting_0.get_value(), 
          stopping_0.get_value()]
        if newvals[0] == True:
          time_stopped = time_stopped + 0.1 
        if newvals[1] == True:
          time_idle = time_idle + 0.1
        if newvals[2] ==True:
          time_starting = time_starting + 0.1
        if newvals[3] ==True:
          time_execute = time_execute+ 0.1
        if newvals[4] ==True:
          time_completing = time_completing + 0.1
        if newvals[5] ==True:
          time_complete = time_complete + 0.1
        if newvals[6] ==True:
          time_clearing = time_clearing + 0.1
        if newvals[7] ==True:
          time_suspended = time_suspended  + 0.1
        if newvals[8] ==True:
          time_aborting = time_aborting + 0.1
        if newvals[9] ==True:
          time_aborted = time_aborted + 0.1
        if newvals[10] ==True:
          time_holding = time_holding + 0.1
        if newvals[11] ==True:
          time_held = time_held  + 0.1
        if newvals[12] ==True:
          time_unholding = time_unholding  + 0.1
        if newvals[13] ==True:
          time_suspending = time_suspending + 0.1
        if newvals[14] ==True:
          time_unsuspending = time_unsuspending  + 0.1
        if newvals[15] ==True:
          time_resetting = time_resetting  + 0.1
        if newvals[16] ==True:
          time_stopping = time_stopping + 0.1
    except KeyboardInterrupt:
      pass
      client.disconnect()

def main(args=None):
  rclpy.init(args=args)
  # Create thread to get data from PLC
  plc_listener = threading.Timer(1.0, PLCListener,args=(thee,))
  plc_listener.start()
  driver_listener = DriverListener()
  rclpy.spin(driver_listener)

if __name__ == '__main__':
	main()
