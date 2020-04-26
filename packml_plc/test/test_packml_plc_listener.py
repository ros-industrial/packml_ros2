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

import threading
import unittest
from unittest.mock import MagicMock

from packml_msgs.srv import AllStatus
from packml_plc.packml_plc_listener import DriverListener
from packml_plc.packml_plc_listener import HelloClient
from packml_plc.packml_plc_listener import main
from packml_plc.packml_plc_listener import newvals, thee
from packml_plc.packml_plc_listener import PLCListener
import rclpy


class TestMethods(unittest.TestCase):

    def test_HelloClient(self):
        client = HelloClient('freeopcua/server/')
        client.client.connect = MagicMock()
        client.client.disconnect = MagicMock()
        client.__enter__()
        client.__exit__(1, 1, 1)

    def test_DriverListener(self):
        rclpy.init(args=None)
        driver = DriverListener()
        driver.create_service = MagicMock()
        self.assertNotEqual(driver.srv, [])
        res = AllStatus.Response()
        driver.sendData(0, res)
        self.assertNotEqual(res, [])
        rclpy.shutdown()

    def test_PLCListener(self):
        HelloClient.__enter__ = MagicMock()
        HelloClient.__exit__ = MagicMock()
        e = threading.Event()
        plc_listener = threading.Timer(1.0, PLCListener, args=(e,))
        plc_listener.start()
        plc_listener.join(5)
        e.set()
        self.assertEqual(newvals, [False, False, False, False, False, False,
                                   False, False, False, False, False, False,
                                   False, False, False, False, False])

    def test_main(self):
        rclpy.spin = MagicMock()
        main()
        thee.set()
