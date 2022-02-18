####################################################################
#   Motion Capture Suit V2                                         #
#   Reads Motion Shadow Data and maps to head's X and Y rotation   #
#   Prompts to choose an arm, the neighbouring arms follows        #
####################################################################

from xml.etree.ElementTree import XML
import time
import sys
from threading import Thread
import queue
import numpy as np
import MotionSDK
import math
import random

host = ""
PortConfigurable = 32076
frames = 0

# Sets xarms to positions in the queue
def playRobot(que, arms):
    while True:
        pos_list = que.get()
        # print(pos_list)
        for i in range(0, 9):
            arms[i].set_servo_angle_j(angles=pos_list[i], is_radian=False)

# BFS Helper algorithm to calculate neighbour distances
def bfs(graph, start):
    discovery = {
        1: -1,
        2: -1,
        3: -1,
        4: -1,
        5: -1,
        6: -1,
        7: -1,
        8: -1,
        9: -1
    }

    Q = queue.Queue()
    visited = set()

    Q.put(start)
    visited.add(start)

    while not Q.empty():
        v = Q.get()
        for u in graph[v]:
            if u not in visited:
                Q.put(u)
                discovery[u] = v
                visited.add(u)

    return discovery

# Calculates weights for each arm given the distance from chosen
def distanceWeights(arm):
    weights = {}

    # neighbor_graph = {
    #     1: [2, 4, 5],
    #     2: [1, 3, 4, 5, 6],
    #     3: [2, 5, 6],
    #     4: [1, 2, 5, 7, 8],
    #     5: [1, 2, 3, 4, 6, 7, 8, 9],
    #     6: [2, 3, 5, 8, 9],
    #     7: [4, 5, 8],
    #     8: [4, 5, 6, 7, 9],
    #     9: [5, 6, 8]
    # }

    neighbor_graph = {
        1: [2, 4],
        2: [1, 3, 5],
        3: [2, 6],
        4: [1, 5, 7],
        5: [2, 4, 6, 8],
        6: [3, 5, 9],
        7: [4, 8],
        8: [5, 7, 9],
        9: [6, 8]
    }

    discovery = bfs(neighbor_graph, arm)

    for i in discovery.keys():
        distance = -1
        j = i
        while j != -1:
            j = discovery.get(j)
            distance += 1

        weight = np.interp(distance, [0, 2], [1, 0.25])

        w1 = int(100*(weight - 0.2))
        w2 = int(100*(weight + 0.2))

        weights[i] = abs(random.randint(w1, w2)/100)

    return weights

def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        a.set_servo_angle(angle=[0.0, 0.0, 0.0, 90, 0.0, 0.0, 0.0], wait=False, speed=20, acceleration=5, is_radian=False)

def parse_name_map(xml_node_list):
    name_map = {}

    tree = XML(xml_node_list)

    # <node key="N" id="Name"> ... </node>
    list = tree.findall(".//node")
    for itr in list:
        name_map[int(itr.get("key"))] = itr.get("id")

    return name_map

def testClient():
    client = MotionSDK.Client(host, PortConfigurable)
    print("Connected to %s:%d" % (host, PortConfigurable))

def data_handler(que, weights):
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

    num_frames = frames
    xml_node_list = None
    head_key = None

    counter = 0

    print(weights)
    weights = list(weights.values())
    print(weights)

    while True:
        data = client.readData()

        if data is None:
            raise RuntimeError("data stream interrupted or timed out")
            break

        if data.startswith(b"<?xml"):
            xml_node_list = data
            continue

        container = MotionSDK.Format.Configurable(data)

        #
        # Consume the XML node name list. If the print header option is active
        # add that now.
        #
        if xml_node_list:
            ChannelName = [
                "Lqw", "Lqx", "Lqy", "Lqz",
                "cw", "cx", "cy", "cz"
            ]

            name_map = parse_name_map(xml_node_list)

            for key in container:
                if key not in name_map:
                    raise RuntimeError(
                        "device missing from name map, unable to print "
                        "header")

                name = name_map[key]

                if name == "Head":
                    head_key = key

        position = []
        rotation = []
        item = container[head_key]

        for i in range(0,3):
            rotation.append((item.value(i)))
        for i in range(4,7):
            position.append((item.value(i)))

        if counter == 0:
            offset0 = rotation[0]
            offset1 = rotation[1]

        mapangle0 = np.interp(math.degrees(rotation[0] - offset0), [-15, 15], [-50, 50])
        mapangle1 = np.interp(math.degrees(rotation[1] - offset1), [-15, 15], [-30, 30])

        if counter > 1000:
            pos_list = []
            for i in range(0, 9):
                # Joint-3 and Joint-6
                # pos = [0.0, 0.0, mapangle1 * weights[i], 90, 0.0,  mapangle0 * weights[i], 0.0]
                # Joint-5 and Joint-6
                pos = [0.0, 0.0, 0, 90, mapangle1 * weights[i],  mapangle0 * weights[i], 0.0]
                pos_list.append(pos)
            que.put(pos_list)

        counter += 1
        num_frames += 1

if __name__ == "__main__":
    ###############
    # Setup Mocap #
    ###############
    # Set the default host name parameter. The SDK is socket based so any
    # networked Motion Service is available.
    if len(sys.argv) > 1:
        host = sys.argv[1]

    ###############
    # Setup xArms #
    ###############
    from libraries.xarm.wrapper import XArmAPI

    ROBOT = "xArms"
    PORT = 5004

    arm1 = XArmAPI('192.168.1.236')
    arm2 = XArmAPI('192.168.1.242')
    arm3 = XArmAPI('192.168.1.203')
    arm4 = XArmAPI('192.168.1.215')
    arm5 = XArmAPI('192.168.1.234')
    arm6 = XArmAPI('192.168.1.244')
    arm7 = XArmAPI('192.168.1.211')
    arm8 = XArmAPI('192.168.1.226')
    arm9 = XArmAPI('192.168.1.208')

    arms = [arm1, arm2, arm3, arm4, arm5, arm6, arm7, arm8, arm9]
    totalArms = len(arms)

    setup()
    repeat = input("do we need to repeat? [y/n]")
    if repeat == 'y':
        setup()
    for a in arms:
        a.set_mode(1)
        a.set_state(0)

    ##################
    # Setup  Threads #
    ##################
    num = int(input("Choose an arm from 1-9:"))
    weights = distanceWeights(num)

    que = queue.Queue()
    t_read = Thread(target=data_handler, args=(que, weights,))
    t_play = Thread(target=playRobot, args=(que, arms,))

    t_read.start()
    t_play.start()




