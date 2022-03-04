from xml.etree.ElementTree import XML
import sys
from threading import Thread
import queue
import numpy as np
import MotionSDK
import math
import random
import time

host = ""
PortConfigurable = 32076
frames = 0


def playRobot(arm, mapangle, joint_angle, weight):
    t_step = 0.006
    tf = 3
    t_array = np.arange(0, tf, t_step)

    p = np.array(arm.angles)

    while True:
        data = mapangle.get()
        j3 = joint_angle.get()
        weight = weight.get()
        j5 = data.get("j5") * weight
        j6 = data.get("j6") * weight
        goal = np.array([0, 0, j3, 0, j5, j6, 0])

        q_i = p
        q_dot_i = np.zeros(7)
        q_dot_f = np.zeros(7)
        q_dotdot_i = np.zeros(7)
        q_dotdot_f = np.zeros(7)
        q_f = goal

        j = 0
        while j < len(t_array):
            start_time = time.time()

            cont = False
            for m, n in zip(p, q_f):
                if (abs(m - n) > 1.0):
                    cont = True
            if cont:
                break

            if j == len(t_array):
                t = tf
            else:
                t = t_array[j]

            a0 = q_i
            a1 = q_dot_i
            a2 = []
            a3 = []
            a4 = []
            a5 = []

            for i in range(0, 7):
                a2.append(0.5 * q_dotdot_i[i])
                a3.append(1.0 / (2.0 * tf ** 3.0) * (20.0 * (q_f[i] - q_i[i]) - (8.0 * q_dot_f[i] + 12.0 * q_dot_i[i]) * tf - (3.0 * q_dotdot_f[i] - q_dotdot_i[i]) * tf ** 2.0))
                a4.append(1.0 / (2.0 * tf ** 4.0) * (30.0 * (q_i[i] - q_f[i]) + (14.0 * q_dot_f[i] + 16.0 * q_dot_i[i]) * tf + (3.0 * q_dotdot_f[i] - 2.0 * q_dotdot_i[i]) * tf ** 2.0))
                a5.append(1.0 / (2.0 * tf ** 5.0) * (12.0 * (q_f[i] - q_i[i]) - (6.0 * q_dot_f[i] + 6.0 * q_dot_i[i]) * tf - (q_dotdot_f[i] - q_dotdot_i[i]) * tf ** 2.0))

                p[i] = (a0[i] + a1[i] * t + a2[i] * t ** 2 + a3[i] * t ** 3 + a4[i] * t ** 4 + a5[i] * t ** 5)

            # arm.set_servo_angle_j(angles=p, is_radian=False)
            print(f"{p} {arm}")

            tts = time.time() - start_time
            sleep = 0.006 - tts

            if tts > 0.006:
                sleep = 0

            time.sleep(sleep)
            j += 1

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

    list = tree.findall(".//node")
    for itr in list:
        name_map[int(itr.get("key"))] = itr.get("id")

    return name_map

def data_handler(mapangle_list, ):
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
            for i in range(len(mapangle_list)):
                mapangle_list[i].put({
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

def findPositionAngle(pos, graph):
    lst = []
    for i in range(len(graph)):
        angle = findAngle(pos, graph[i])
        lst.append(angle)
        # print(f"arm: {i+1}, coordinate {graph[i]}, relative angle: {angle}")
    return lst

def closest_arm(pos, nodes):
    # nodes = np.asarray(nodes)
    deltas = nodes - pos
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    return np.argmin(dist_2)

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

def getPosition(pos_que,):
    print("Enter dancer coordinates (floating number)")
    x = float(input("X: "))
    y = float(input("Y: "))
    pos = [x, y]
    pos_que.put(pos)

def updateWeights(pos_que, w_list, j_list, graph):
    while True:
        pos = pos_que.get()
        num = closest_arm(pos, graph)
        leader = graph[num]

        weights = findWeights(leader, graph)
        j_angles = findPositionAngle(pos, graph)

        for i in range(len(graph)):
            j_list[i].put(j_angles[i])
            w_list[i].put(weights[i])


if __name__ == "__main__":
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

    graph = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [0.0, 1.0], [1.0, 1.0], [2.0, 1.0], [0.0, 2.0], [1.0, 2.0], [2.0, 2.0]])

    pos_que = queue.Queue()
    mapangle_list = []
    j_list = []
    w_list = []

    for i in range(len(graph)):
        mapangle_list.append(queue.Queue)
        j_list.append(queue.Queue)
        w_list.append(queue.Queue)

    t_position = Thread(target=getPosition, args=(pos_que,))
    t_update = Thread(target=updateWeights, args=(pos_que, w_list, j_list, graph,))
    t_mocap = Thread(target=data_handler, args=(mapangle_list,))
    t_arms = []
    for i in range(len(graph)):
        t_arms.append(Thread(target=playRobot, args=(arms[i], mapangle_list[i], j_list[i], w_list[i])))


    t_position.start()
    t_update.start()
    t_mocap.start()
    for i in range(len(graph)):
        t_arms[i].start()