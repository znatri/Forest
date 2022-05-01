from xml.etree.ElementTree import XML
import sys
from threading import Thread
import queue
import numpy as np
import MotionSDK
import math
import random
import time
import socket

host = ""
PortConfigurable = 32076
frames = 0

def playRobot(arm, map_angle : queue.Queue, joint_angle : queue.Queue, weight_que: queue.Queue):
    tf_pos = 50
    tf_shadow = 5
    t_step = 0.006
    t_array_pos = np.arange(0, tf_pos, t_step)
    t_array_shadow = np.arange(0, tf_shadow, t_step)

    q_dot_f = np.zeros(7)
    q_dotdot_f = np.zeros(7)
    p = [0, 0, 0, 90, 0, 0, 0]

    while True:
        data = map_angle.get()
        if not weight_que.empty():
            weight = weight_que.get()

        if not joint_angle.empty():
            j3 = joint_angle.get()
        else:
            j3 = p[2]
        j5 = data.get("j5") * weight
        j6 = data.get("j6") * weight
        # j3 = 0
        # j5 = data.get("j5")
        # j6 = data.get("j6")
        goal = [0, 0, j3, 90, j5, j6, 0]

        q_i = p
        q_dot_i = np.zeros(7)
        q_dotdot_i = np.zeros(7)
        q_f = goal

        j = 0
        k = 0

        while j < len(t_array_pos) or k < len(t_array_shadow):
            start_time = time.time()

            if abs(p[0] - q_f[0]) < 1.0 and abs(p[1] - q_f[1]) < 1.0 and abs(p[2] - q_f[2]) < 1.0 and abs(
                    p[3] - q_f[3]) < 1.0 and abs(p[4] - q_f[4]) < 1.0 and abs(p[5] - q_f[5]) < 1.0 and abs(
                p[6] - q_f[6]) < 1.0:
                map_angle.queue.clear()
                # joint_angle.queue.clear()
                break

            if j >= len(t_array_pos):
                # joint_angle.queue.clear()
                t_pos = tf_pos
            else:
                t_pos = t_array_pos[j]

            if k >= len(t_array_shadow):
                map_angle.queue.clear()
                t_shadow = tf_shadow
            else:
                t_shadow = t_array_shadow[k]

            a0 = q_i
            a1 = q_dot_i
            a2 = []
            a3 = []
            a4 = []
            a5 = []

            for i in range(0, 7):

                if i == 4 or i == 5:
                    tf = tf_shadow
                    t = t_shadow
                else:
                    tf = tf_pos
                    t = t_pos

                a2.append(0.5 * q_dotdot_i[i])
                a3.append(1.0 / (2.0 * tf ** 3.0) * (
                        20.0 * (q_f[i] - q_i[i]) - (8.0 * q_dot_f[i] + 12.0 * q_dot_i[i]) * tf - (
                        3.0 * q_dotdot_f[i] - q_dotdot_i[i]) * tf ** 2.0))
                a4.append(1.0 / (2.0 * tf ** 4.0) * (
                        30.0 * (q_i[i] - q_f[i]) + (14.0 * q_dot_f[i] + 16.0 * q_dot_i[i]) * tf + (
                        3.0 * q_dotdot_f[i] - 2.0 * q_dotdot_i[i]) * tf ** 2.0))
                a5.append(1.0 / (2.0 * tf ** 5.0) * (
                        12.0 * (q_f[i] - q_i[i]) - (6.0 * q_dot_f[i] + 6.0 * q_dot_i[i]) * tf - (
                        q_dotdot_f[i] - q_dotdot_i[i]) * tf ** 2.0))

                p[i] = (a0[i] + a1[i] * t + a2[i] * t ** 2 + a3[i] * t ** 3 + a4[i] * t ** 4 + a5[i] * t ** 5)

            arm.set_servo_angle_j(angles=p, is_radian=False)
            # print(f"{p} {arm}")

            tts = time.time() - start_time
            sleep = t_step - tts

            if tts > t_step:
                sleep = 0

            time.sleep(sleep)
            j += 1
            k += 1


def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        # a.motion_enable(enable=True)
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

# MOCAP Data handle
def data_handler(mapangle_ques,):
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

    mapangles = [[], []]

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

        mapangles[0].append(np.interp(math.degrees(rotation[1] - offset1), [-15, 15], [-50, 50]))
        mapangles[1].append(np.interp(math.degrees(rotation[0] - offset0), [-15, 15], [-70, 70]))

        windows = [mapangles[0], mapangles[1]]

        if counter >= 2500:
            j = []
            for joint in range(len(windows)):
                windows[joint] = windows[joint][-30:]
                j.append(np.mean(windows[joint]))

            # print(f'{j[0]}, {j[1]}')

            for que in mapangle_ques:
                que.put({
                    'j5': j[0],
                    'j6': j[1],
                })

        # if counter > 500:
        #     for que in mapangle_ques:
        #         que.put({
        #             'j5': mapangle0,
        #             'j6': mapangle1,
        #         })

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

# Angle for robot to face dancer
def findPositionAngle(pos, graph):
    # print(pos)
    lst = []
    for i in range(len(graph)):
        angle = findAngle(pos, graph[i])
        lst.append(angle)
        # print(f"arm: {i+1}, coordinate {graph[i]}, relative angle: {angle}")
    return lst

# Finds the closest arm
def closest_arm(pos, nodes):
    # nodes = np.asarray(nodes)
    deltas = nodes - pos
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    retVal = np.argmin(dist_2)
    print(retVal)
    return np.argmin(dist_2)

# Calculates weight for an arm
def findWeights(originbot, otherbots):
    randomizedWeights = []

    distances = findDistances(originbot, otherbots)

    max = np.max(distances)
    min = np.min(distances)

    # print(f"Max: {max}, Min: {min}")

    weights = np.interp(distances, [min, max], [1, 0.25])

    for i in weights:
        w1 = int(100 * (i - 0.2))
        w2 = int(100 * (i + 0.2))

        randomizedWeights.append(abs(random.randint(w1, w2) / 100))

    return randomizedWeights

# Get dancer position from input
# def getPosition(pos_que, ):
#     while True:
#         while True:
#             try:
#                 x = float(input("X:"))
#                 break
#             except ValueError:
#                 continue
#
#         while True:
#             try:
#                 y = float(input("Y:"))
#                 break
#             except ValueError:
#                 continue
#
#         pos = [x, y]
#         pos_que.put(pos)

# Get dancer position from MAX patch
def getDancerPos(pos_que, ):
    MAX_UDP_IP = "127.0.0.1"
    MAX_UDP_PORT = 7983

    s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    s.bind((MAX_UDP_IP, MAX_UDP_PORT))
    s.settimeout(None)

    try:
        while True:
            data, server = s.recvfrom(1024)
            coord = bytes.decode(data).split()
            pos = [float(coord[0]), float(coord[1])]
            pos_que.put(pos)
            # print(float(coord[0]), float(coord[1]))
    except KeyboardInterrupt or socket.error:
        s.close()
    finally:
        s.close()
        print("Socket closed")

# Updates weights in thread
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
    from xarm.wrapper import XArmAPI

    ROBOT = "xArms"
    PORT = 5004

    arm1 = XArmAPI('192.168.1.203')
    arm2 = XArmAPI('192.168.1.242')
    arm3 = XArmAPI('192.168.1.236')
    arm4 = XArmAPI('192.168.1.244')
    arm5 = XArmAPI('192.168.1.234')
    arm6 = XArmAPI('192.168.1.215')

    # arm7 = XArmAPI('192.168.1.208')
    # arm8 = XArmAPI('192.168.1.226')
    # arm9 = XArmAPI('192.168.1.211')

    # arms = [arm1, arm2, arm3, arm4, arm5, arm6, arm7, arm8, arm9]
    arms = [arm1, arm2, arm3, arm4, arm5, arm6]
    # arms = [1, 2, 3, 4, 5, 6]
    totalArms = len(arms)

    setup()
    repeat = input("do we need to repeat? [y/n]")
    if repeat == 'y':
        setup()
    for a in arms:
        a.set_mode(1)
        a.set_state(0)

    # graph = np.array( [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [0.0, 1.0], [1.0, 1.0], [2.0, 1.0], [0.0, 2.0], [1.0, 2.0], [2.0, 2.0]])

    # graph_posenet = np.array(
    #     [[1050.0, 380.0], [710.0, 252.0], [410.0, 115.0], [1180.0, 290.0], [900.0, 200.0], [630.0, 100.0], [1275.0, 250.0], [1010.0, 175.0], [810.0, 85.0]])

    graph_posenet = np.array([[1050.0, 380.0], [710.0, 252.0], [410.0, 115.0], [1180.0, 290.0], [900.0, 200.0], [630.0, 100.0]])

    pos_que = queue.Queue()
    mapangle_ques = []
    j_list = []
    w_list = []

    for i in range(len(graph_posenet)):
        mapangle_ques.append(queue.Queue())
        j_list.append(queue.Queue())
        w_list.append(queue.Queue())

    # t_position = Thread(target=getPosition, args=(pos_que,))
    t_position = Thread(target=getDancerPos, args=(pos_que,))
    t_update = Thread(target=updateWeights, args=(pos_que, w_list, j_list, graph_posenet,))
    t_mocap = Thread(target=data_handler, args=(mapangle_ques,))
    t_arms = []
    for i in range(totalArms):
        t_arms.append(Thread(target=playRobot, args=(arms[i], mapangle_ques[i], j_list[i], w_list[i])))

    t_position.start()
    t_update.start()
    # t_mocap.start()
    for i in range(totalArms):
        t_arms[i].start()