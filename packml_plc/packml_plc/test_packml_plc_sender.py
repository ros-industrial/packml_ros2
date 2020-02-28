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
from packml_plc_sender import *
from opcua import Client, ua
from opcua.ua import ua_binary as uabin
from opcua.common.methods import call_method
from opcua import common
from packml_msgs.srv import Transition
import time
import rclpy 
from rclpy.node import Node
import unittest
from mock import MagicMock, Mock

class TestMethods(unittest.TestCase):
  def test_DriverSender(self):
    Client = MagicMock()
    rclpy.init(args=None)
    driver = DriverSender()
    self.assertNotEqual(driver.client,[])
    rclpy.shutdown()
  def test_DriverSender_connect(self):
    Client = MagicMock()
    rclpy.init(args=None)
    driver = DriverSender()
    driver.client.connect = MagicMock()
    driver.connect()
    rclpy.shutdown()
  def test_DriverSender_exit(self):
    Client = MagicMock()
    rclpy.init(args=None)
    driver = DriverSender()
    driver.client.disconnect = MagicMock()
    driver.__exit__(1,1,1)
    rclpy.shutdown()
  def test_DriverSender_transitonRequest(self):
    Client = MagicMock()
    rclpy.init(args=None)
    driver = DriverSender()
    driver.client.get_node = MagicMock()
    res = Transition.Response()
    req = Transition.Request()
    req.command = 1 
    driver.transRequest(req,res)
    self.assertNotEqual(res.success,False)
    req.command = 2
    driver.transRequest(req,res)
    self.assertNotEqual(res.success,False)
    req.command = 3
    driver.transRequest(req,res)
    self.assertNotEqual(res.success,False)
    req.command = 4
    driver.transRequest(req,res)
    self.assertNotEqual(res.success,False)
    req.command = 5
    driver.transRequest(req,res)
    self.assertNotEqual(res.success,False)
    req.command = 6
    driver.transRequest(req,res)
    self.assertNotEqual(res.success,False)
    req.command = 7
    driver.transRequest(req,res)
    self.assertNotEqual(res.success,False)
    req.command = 100
    driver.transRequest(req,res)
    self.assertNotEqual(res.success,False)
    req.command = 101
    driver.transRequest(req,res)
    self.assertNotEqual(res.success,False)
    req.command = 102
    driver.transRequest(req,res)
    self.assertNotEqual(res.success,False)
    req.command = 103
    driver.transRequest(req,res)
    self.assertNotEqual(res.success,True)
    rclpy.shutdown()

suite = unittest.TestLoader().loadTestsFromTestCase(TestMethods)
unittest.TextTestRunner(verbosity=2).run(suite)
