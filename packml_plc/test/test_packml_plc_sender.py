#!/usr/bin/env python3
#
# Software License Agreement
# Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific
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
import unittest
from unittest.mock import MagicMock

from packml_msgs.srv import Transition
from packml_plc.packml_plc_sender import DriverSender


class TestMethods(unittest.TestCase):

    def test_driversender(self):
        driver = DriverSender()
        self.assertNotEqual(driver.client, [])

    def test_driversender_connect(self):
        driver = DriverSender()
        driver.client.connect = MagicMock()
        driver.connect()

    def test_driversender_exit(self):
        driver = DriverSender()
        driver.client.disconnect = MagicMock()
        driver.__exit__(1, 1, 1)

    def test_driversender_trans_request(self):
        driver = DriverSender()
        driver.client.get_node = MagicMock()
        res = Transition.Response()
        req = Transition.Request()
        req.command = 1
        driver.trans_request(req, res)
        self.assertNotEqual(res.success, False)
        req.command = 2
        driver.trans_request(req, res)
        self.assertNotEqual(res.success, False)
        req.command = 3
        driver.trans_request(req, res)
        self.assertNotEqual(res.success, False)
        req.command = 4
        driver.trans_request(req, res)
        self.assertNotEqual(res.success, False)
        req.command = 5
        driver.trans_request(req, res)
        self.assertNotEqual(res.success, False)
        req.command = 6
        driver.trans_request(req, res)
        self.assertNotEqual(res.success, False)
        req.command = 7
        driver.trans_request(req, res)
        self.assertNotEqual(res.success, False)
        req.command = 100
        driver.trans_request(req, res)
        self.assertNotEqual(res.success, False)
        req.command = 101
        driver.trans_request(req, res)
        self.assertNotEqual(res.success, False)
        req.command = 102
        driver.trans_request(req, res)
        self.assertNotEqual(res.success, False)
        req.command = 103
        driver.trans_request(req, res)
        self.assertNotEqual(res.success, True)
