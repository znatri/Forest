####################################################################
#   Motion Capture Suit Parser V3                                  #
#   Reads Motion Shadow Data and maps to head's X and Y rotation   #
#   Prompts to choose an arm, the neighbouring arms follows        #
####################################################################

from xml.etree.ElementTree import XML
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
def playRobot(arm, que, j3, weight):
    while True:
        data = que.get()
        j5 = data.get("j5") * weight
        j6 = data.get("j6") * weight
        arm.set_servo_angle_j(angles=[0.0, 0.0, j3, 90, j5, j6, 0.0], is_radian=False)


def findPositionAngle(pos, graph):
    lst = []
    for i in range(len(graph)):
        angle = findAngle(pos, graph[i])
        lst.append(angle)
        # print(f"arm: {i+1}, coordinate {graph[i]}, relative angle: {angle}")
    return lst


def findWeights(originbot, otherbots):
    randomizedWeights = []

    distances = findDistances(originbot, otherbots)

    max = np.max(distances)
    min = np.min(distances)

    print(f"Max: {max}, Min: {min}")

    weights = np.interp(distances, [min, max], [1, 0.25])

    for i in weights:
        w1 = int(100 * (i - 0.2))
        w2 = int(100 * (i + 0.2))

        randomizedWeights.append(abs(random.randint(w1, w2) / 100))

    return randomizedWeights


def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        a.set_servo_angle(angle=[0.0, 0.0, 0.0, 90, 0.0, 0.0, 0.0], wait=False, speed=20, acceleration=5,
                          is_radian=False)


def parse_name_map(xml_node_list):
    name_map = {}

    tree = XML(xml_node_list)

    # <node key="N" id="Name"> ... </node>
    list = tree.findall(".//node")
    for itr in list:
        name_map[int(itr.get("key"))] = itr.get("id")

    return name_map


def data_handler(que, ):
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

    while True:
        data = client.readData()

        if data is None:
            raise RuntimeError("data stream interrupted or timed out")

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

        for i in range(0, 3):
            rotation.append((item.value(i)))
        for i in range(4, 7):
            position.append((item.value(i)))

        if counter <= 500:
            offset0 = rotation[0]
            offset1 = rotation[1]

        mapangle0 = np.interp(math.degrees(rotation[0] - offset0), [-15, 15], [-50, 50])
        mapangle1 = np.interp(math.degrees(rotation[1] - offset1), [-15, 15], [-30, 30])

        if counter > 500:
            for i in range(9):
                que[i].put({
                    'j5': mapangle0,
                    'j6': mapangle1,
                })

        counter += 1
        num_frames += 1


def findDistance(origin, newpoint):
    distance = ((((newpoint[0] - origin[0]) ** 2) + ((newpoint[1] - origin[1]) ** 2)) ** 0.5)
    return distance


def findDistances(pos, nodes):
    deltas = nodes - pos
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    distances = np.sqrt(dist_2)
    return distances


def findAngle(pos, arm_pos):
    (x, y) = pos
    (a, b) = arm_pos

    if (x, y) == (a, b):
        return 0

    opposite = findDistance((x, y), (a, y))
    hypotenuse = findDistance((x, y), (a, b))
    angleRad = math.asin(opposite / hypotenuse)
    angle = 180 * angleRad / math.pi

    if x <= a:
        if y <= b:
            angle = -angle
        elif y > b:
            angle = -(180 - angle)
    elif x > a:
        if y <= b:
            angle = angle
        elif y > b:
            angle = (180 - angle)

    return angle


def closest_arm(pos, nodes):
    # nodes = np.asarray(nodes)
    deltas = nodes - pos
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    return np.argmin(dist_2)


if __name__ == "__main__":
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

    #################################
    # Get location and find weights #
    ################################

    graph = np.array(
        [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [0.0, 1.0], [1.0, 1.0], [2.0, 1.0], [0.0, 2.0], [1.0, 2.0], [2.0, 2.0]])

    print("Enter dancer coordinates (floating number)")
    x = float(input("X: "))
    y = float(input("Y: "))
    pos = [x, y]

    # Calculate the closest arm's position using dancer position
    num = closest_arm(pos, graph)

    ################################
    # Setup xArm and MoCap Threads #
    ###############################

    # Store weights for each arms in a sequential order, 1 ... 9
    leader = graph[num]
    weights = findWeights(leader, graph)

    # Values for joint 3 from current position
    j3_values = findPositionAngle(pos, graph)

    # Store angles from mocap data
    que = []
    for i in range(len(graph)):
        que.append(queue.Queue())

    # Mocap and arms handler threads
    t_mocap = Thread(target=data_handler, args=(que,))
    t_arms = []

    for i in range(len(graph)):
        t_arms.append(Thread(target=playRobot, args=(arms[i], que[i], j3_values[i], weights[i],)))

    ###########
    # Execute #
    ##########

    input("Press enter to execute:\n")

    t_mocap.start()

    for i in range(len(graph)):
        t_arms[i].start()

