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
# Contributors: Dejanira Araiza Illan
#

from packml_plc_listener import *
from opcua import Client, ua
from opcua.ua import ua_binary as uabin
from opcua.common.methods import call_method
from opcua import common
from packml_msgs.srv import Transition
import time
import rclpy 
from rclpy.node import Node
import unittest
from unittest.mock import MagicMock, Mock
from packml_msgs.srv import AllStatus
import socket

class TestMethods(unittest.TestCase):
  def test_HelloClient(self):
    Client = MagicMock()
    client = HelloClient("freeopcua/server/")
    client.client.connect = MagicMock()
    client.client.disconnect = MagicMock()
    client.__enter__()
    client.__exit__(1,1,1)

  def test_DriverListener(self):
    rclpy.init(args=None)
    driver = DriverListener()
    driver.create_service = MagicMock()
    self.assertNotEqual(driver.srv,[])
    res=AllStatus.Response()
    driver.sendData(0,res)
    self.assertNotEqual(res,[])
    rclpy.shutdown()

  def test_PLCListener(self):
    Client = MagicMock()
    HelloClient.__enter__ = MagicMock()
    HelloClient.__exit__ = MagicMock()
    e = threading.Event()
    plc_listener = threading.Timer(1.0, PLCListener,args=(e,))
    plc_listener.start()
    plc_listener.join(5)
    e.set()
    self.assertEqual(newvals,[False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False])
    
  def test_main(self):
    PLCListener = MagicMock()
    DriverListener = MagicMock()
    rclpy.spin = MagicMock()
    main()
    thee.set()

suite = unittest.TestLoader().loadTestsFromTestCase(TestMethods)
unittest.TextTestRunner(verbosity=2).run(suite)
