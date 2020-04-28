#!/usr/bin/env python
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
# Contributors: Dejanira Araiza Illan, Derrick Ang Ming Yan
#

import time

from opcua import Client, ua
from packml_msgs.srv import Transition
import rclpy
from rclpy.node import Node


class DriverSender(Node):
    """This class controls the PLC's SM transition by pressing buttons on the GUI."""

    def __init__(self):
        super().__init__('driver_sender')
        self.srv = self.create_service(Transition, 'transition', self.trans_request)
        self.client = Client('opc.tcp://192.168.125.2:4840/freeopcua/server/')

    def connect(self):
        self.client.connect()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.client.disconnect()

    def trans_request(self, req, res):
        command_rtn = False
        command_valid = True
        command_int = req.command
        print('Evaluating transition request command: ' + str(command_int))
        try:
            if command_int == 5:
                cmd_abort = self.client.get_node('ns=3;' +
                                                 's=\"PackML_Status\".\"EM00\"' +
                                                 '.\"Unit\".\"Cmd_Abort\"')
                cmd_abort.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                time.sleep(0.1)
                print('Sending Abort Command . . .')
                command_rtn = True
            elif command_int == 7:
                cmd_stop = self.client.get_node('ns=3;' +
                                                's=\"PackML_Status\".\"EM00\".' +
                                                '\"Unit\".\"Cmd_Stop\""')
                cmd_stop.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                time.sleep(0.1)
                print('Sending Stop Command . . .')
                command_rtn = True
            elif command_int == 1:
                cmd_clear = self.client.get_node('ns=3;' +
                                                 's=\"PackML_Status\".\"EM00\".' +
                                                 '\"Unit\".\"Cmd_Clear\"')
                cmd_clear.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                time.sleep(0.1)
                print('Sending Clear Command . . .')
                command_rtn = True
            elif command_int == 4:
                cmd_hold = self.client.get_node('ns=3;' +
                                                's=\"PackML_Status\".\"EM00\".' +
                                                '\"Unit\".\"Cmd_Hold\"')
                cmd_hold.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                time.sleep(0.1)
                print('Sending Hold Command . . .')
                command_rtn = True
            elif command_int == 6:
                cmd_reset = self.client.get_node('ns=3;' +
                                                 's=\"PackML_Status\".\"EM00\".' +
                                                 '\"Unit\".\"Cmd_Reset\"')
                cmd_reset.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                time.sleep(0.1)
                print('Sending Reset Command . . .')
                command_rtn = True
            elif command_int == 2:
                cmd_start = self.client.get_node('ns=3;' +
                                                 's=\"PackML_Status\".\"EM00\".' +
                                                 '\"Unit\".\"Cmd_Start\"')
                cmd_start.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                time.sleep(0.1)
                print('Sending Start Command . . .')
                command_rtn = True
            elif command_int == 3:
                cmd_stop = self.client.get_node('ns=3;' +
                                                's=\"PackML_Status\".\"EM00\".' +
                                                '\"Unit\".\"Cmd_Stop\"')
                cmd_stop.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                time.sleep(0.05)
                print('Sending Stop Command . . .')
                command_rtn = True
            elif command_int == 100:
                cmd_suspend = self.client.get_node('ns=3;' +
                                                   's=\"PackML_Status\".\"EM00\".' +
                                                   '\"Unit\".\"Cmd_Suspend\"')
                cmd_suspend.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                time.sleep(0.1)
                print('Sending Suspend Command . . .')
                command_rtn = True
            elif command_int == 102:
                cmd_unhold = self.client.get_node('ns=3;' +
                                                  's=\"PackML_Status\".\"EM00\".' +
                                                  '\"Unit\".\"Cmd_Unhold\"')
                cmd_unhold.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                time.sleep(0.1)
                print('Sending Unhold Command . . .')
                command_rtn = True
            elif command_int == 101:
                cmd_unsuspend = self.client.get_node('ns=3;' +
                                                     's=\"PackML_Status\".\"EM00\".' +
                                                     '\"Unit\".\"Cmd_Unsuspend\"')
                cmd_unsuspend.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                time.sleep(0.1)
                print('Sending Unsuspend Command . . .')
                command_rtn = True
            else:
                command_valid = False
        except KeyboardInterrupt:
            pass
        if command_valid:
            if command_rtn:
                print('Successful transition request command: ' + str(command_int))
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
    driver_sender = DriverSender()
    driver_sender.connect()
    rclpy.spin(driver_sender)


if __name__ == '__main__':
    main()
