##################################################################
#   Motion Capture Suit V1                                       #
#   Reads Motion Shadow Data and maps to head's X and Y rotation #
#   Moves one preselected arm                                    #
##################################################################

from xml.etree.ElementTree import XML
import time
import sys
from threading import Thread
import queue
import numpy as np
import MotionSDK
import math

host = ""
PortConfigurable = 32076
frames = 0

def playRobot(que, arm):
    IP = [0, 0, 0, 90, 0, 0, 0]
    tf = 2

    # t0 = 0
    t = [0 for i in range(0, 7)]
    q_i = [0 for i in range(0, 7)]
    q_dot_i = [0 for i in range(0, 7)]
    q_dot_f = [0 for i in range(0, 7)]
    q_dotdot_i = [0 for i in range(0, 7)]
    q_dotdot_f = [0 for i in range(0, 7)]
    t_array = np.arange(0, tf, 0.006)
    p = [0, 0, 0, 90, 0, 0, 0]
    v = [0 for i in range(0, 7)]
    a = [0 for i in range(0, 7)]

    while True:
        q = que.get()
        goal = q
        q_i = p
        q_dot_i = [0, 0, 0, 0, 0, 0, 0]
        q_dotdot_i = [0, 0, 0, 0, 0, 0, 0]
        q_f = goal
        j = 0

        while j < len(t_array):
            start_time = time.time()

            if abs(p[0] - q_f[0]) < 1.0 and abs(p[1] - q_f[1]) < 1.0 and abs(p[2] - q_f[2]) < 1.0 and abs(
                    p[3] - q_f[3]) < 1.0 and abs(p[4] - q_f[4]) < 1.0 and abs(p[5] - q_f[5]) < 1.0 and abs(
                p[6] - q_f[6]) < 1.0:
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
                v[i] = (a1[i] + 2 * a2[i] * t + 3 * a3[i] * t ** 2 + 4 * a4[i] * t ** 3 + 5 * a5[i] * t ** 4)
                a[i] = (2 * a2[i] + 6 * a3[i] * t + 12 * a4[i] * t ** 2 + 20 * a5[i] * t ** 3)

            # arm.set_servo_angle_j(angles=p, is_radian=False)
            # print(f"{p} {arm}")

            print(f"{p}")

            tts = time.time() - start_time
            sleep = 0.006 - tts

            if tts > 0.006:
                sleep = 0

            time.sleep(sleep)
            j += 1

def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        # a.motion_enable(enable=True)
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

def data_reader(que, arm):
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
        mapangle1 = np.interp(math.degrees(rotation[1] - offset1), [-15, 15], [-10, 10])

        if counter > 1000:
            pos = [0.0, 0.0, mapangle1, 90, 0.0, mapangle0, 0.0]
            arm.set_servo_angle_j(angles=pos, is_radian=False)
            # que.put(pos)

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

    arm1 = XArmAPI('192.168.1.242')

    arms = [arm1]
    totalArms = len(arms)

    setup()
    repeat = input("do we need to repeat? [y/n]")
    if repeat == 'y':
        setup()
    for a in arms:
        a.set_mode(1)
        a.set_state(0)

    #################################
    # Setup Live Trajectory Threads #
    #################################
    arm1_q = queue.Queue()
    que_pos = [arm1_q]

    # Threads
    t_read = Thread(target=data_reader, args=(arm1_q, arm1,))
    t_arm = Thread(target=playRobot, args=(arm1_q, arm1))

    # Start Threads
    t_read.start()
    # t_arm.start()



