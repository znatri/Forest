#
# Simple test program for the Python Motion SDK.
#
# @file    sdk/python/test.py
# @version 2.5
#
# Copyright (c) 2017, Motion Workshop
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
import sys
import MotionSDK
from xml.etree.ElementTree import XML
import time
from xarm.wrapper import XArmAPI
from playsound import playsound
import os
import csv
import sys
from queue import Queue
from threading import Thread
import random

PortConsole = 32075
PortConfigurable = 32076
NSample = 10


def parse_name_map(xml_node_list):
    name_map = {}

    tree = XML(xml_node_list)

    # <node key="N" id="Name"> ... </node>
    list = tree.findall(".//node")
    for itr in list:
        name_map[int(itr.get("key"))] = itr.get("id")

    return name_map


def test_Client(host):
    client = MotionSDK.Client(host, PortConfigurable)

    print("Connected to %s:%d" % (host, PortConfigurable))

    xml_string = \
        "<?xml version=\"1.0\"?>" \
        "<configurable inactive=\"1\">" \
        "<r/>" \
        "<c/>" \
        "</configurable>"

    if not client.writeData(xml_string):
        raise RuntimeError(
            "failed to send channel list request to Configurable service")


    num_frames = 0
    xml_node_list = None
    hips_key = None
    hips2_key = None


    lastpoint = 0.00
    counter = 0
    t_elapsed = 0
    arm = XArmAPI('192.168.1.244')

    arm.set_simulation_robot(on_off=False)
    # a.motion_enable(enable=True)
    arm.clean_warn()
    arm.clean_error()
    arm.set_mode(0)
    arm.set_state(0)

    arm.set_servo_angle(angle=[0.0, -1.0, 0.0, 1.309, 0.0, 0.88, 0.0], wait=True, speed=0.4, acceleration=0.25, is_radian=True)

    arm.set_mode(1)
    arm.set_state(0)
    input("press enter to begin")
    time.sleep(5)
    while True:
        # Block, waiting for the next sample.
        data = client.readData()
        if data is None:
            raise RuntimeError("data stream interrupted or timed out")
            break

        if data.startswith(b"<?xml"):
            xml_node_list = data
            continue

        container = MotionSDK.Format.Configurable(data)

        #
        # Consume the XML node name list.
        #
        if xml_node_list:
            name_map = parse_name_map(xml_node_list)

            flat_list = []
            for key in container:
                if key not in name_map:
                    raise RuntimeError(
                        "device missing from name map, unable to print "
                        "header")

                item = container[key]
                item2 = container[key]

                name = name_map[key]
                if "Head" == name:
                    hips_key = key
                if "Hips" == name:
                    hips2_key = key





                print("{}:{}".format(key, name))

            #print(",".join(["{}".format(v) for v in flat_list]))

            if None == hips_key:
                raise RuntimeError("Hips sensor missing from name map")

            xml_node_list = None

        #
        # get the rotation and position values
        #
        position = []
        rotation = []
        position2 = []
        rotation2 = []
        item = container[hips_key]
        item2 = container[hips2_key]

        # from https://www.motionshadow.com/download/media/configurable.xml
        # the channels requested come in order.  we requested r, c
        # r = [x,y,z]  Euler angle in radians
        # c = [w,x,y,z]  w is the constraint value (0 or 1, can ignore)
        # so, r = values [0:3], c = values [4,7] (ignore w value)

        # 1: Body
        # 2: Chest
        # 3: Head
        # 4: HeadEnd
        # 5: Hips
        # 6: LeftArm
        # 7: LeftFinger
        # 8: LeftFingerEnd
        # 9: LeftFoot
        # 10: LeftForearm
        # 11: LeftHand
        # 12: LeftHeel
        # 13: LeftLeg
        # 14: LeftShoulder
        # 15: LeftThigh
        # 16: LeftToe
        # 17: LeftToeEnd
        # 18: Neck
        # 19: RightArm
        # 20: RightFinger
        # 21: RightFingerEnd
        # 22: RightFoot
        # 23: RightForearm
        # 24: RightHand
        # 25: RightHeel
        # 26: RightLeg
        # 27: RightShoulder
        # 28: RightThigh
        # 29: RightToe
        # 30: RightToeEnd
        # 31: SpineLow
        for i in range(0,3):
            rotation.append((item.value(i)))
            rotation2.append((item2.value(i)))
        for i in range(4,7):
            position.append(item.value(i))
            position2.append(item2.value(i))
        if counter == 0:
            offset = rotation[0]
            offset2 = rotation[1]
            offset3 = rotation[1]



        mapangle = rotation[0] - offset
        mapangle2 = rotation[1] -offset2
        mapangle3 = rotation2[1] - offset3
        # if abs(mapangle-lastpoint) > 0.001:
        #     mapangle = lastpoint+0.001
        print(mapangle)
        arm.set_servo_angle_j(angles=[0.0, -1.0, 0.0, 1.309, mapangle2, 0.88 + mapangle3, 0.0], is_radian=True)
        # lastpoint = mapangle
        counter += 1




        #print("r: " + ",".join(["{}".format(round(v, 8)) for v in rotation]))
        # print("c: " + ",".join(["{}".format(round(v, 8)) for v in position]))

        num_frames += 1





def test_LuaConsole(host):
    client = MotionSDK.Client(host, PortConsole)

    print("Connected to %s:%d" % (host, port))

    #
    # General Lua scripting interface.
    #
    lua_chunk = \
      "if not node.is_reading() then" \
      "   node.close()" \
      "   node.scan()" \
      "   node.start()" \
      " end" \
      " if node.is_reading() then" \
      "   print('Reading from ' .. node.num_reading() .. ' device(s)')" \
      " else" \
      "   print('Failed to start reading')" \
      " end"

    print(LuaConsole.SendChunk(client, lua_chunk, 5))

    # Scripting language compatibility class. Translate Python calls into Lua
    # calls and send them to the console service.
    node = LuaConsole.Node(client)
    print("node.is_reading() = {}".format(node.is_reading()))




def main(argv):
    # Set the default host name parameter. The SDK is socket based so any
    # networked Motion Service is available.
    host = ""
    if len(argv) > 1:
        host = argv[1]

    #test_LuaConsole(host)
    test_Client(host)

    # Requires a data file. Do not test by default.
    #test_File()

if __name__ == "__main__":

    sys.exit(main(sys.argv))