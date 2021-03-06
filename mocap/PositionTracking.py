from threading import Thread
import queue
import numpy as np
import math
import time

def playRobot(que, arm):
    tf = 50
    t_step = 0.006
    t_array = np.arange(0, tf, t_step)

    q_dot_f = np.zeros(7)
    q_dotdot_f = np.zeros(7)
    p = [0, 0, 0, 90, 0, 0, 0]

    while True:
        q = que.get()

        goal = q

        q_i = p
        q_dot_i = np.zeros(7)
        q_dotdot_i = np.zeros(7)
        q_f = goal

        j = 0

        while j < len(t_array):
            start_time = time.time()

            if abs(p[0] - q_f[0]) < 2.0 and abs(p[1] - q_f[1]) < 2.0 and abs(p[2] - q_f[2]) < 2.0 and abs(
                    p[3] - q_f[3]) < 2.0 and abs(p[4] - q_f[4]) < 2.0 and abs(p[5] - q_f[5]) < 2.0 and abs(
                p[6] - q_f[6]) < 2.0:
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

def findDistance(origin, newpoint):
    distance = ((((newpoint[0] - origin[0]) ** 2) + ((newpoint[1] - origin[1]) ** 2)) ** 0.5)
    return distance

def findPositionAngle(pos, graph):
    lst = []
    for i in range(len(graph)):
        angle = findAngle(pos, graph[i])
        lst.append(angle)
        # print(f"arm: {i+1}, coordinate {graph[i]}, relative angle: {angle}")
    return lst

def setup():
    for a in arms:
        if a != 0:
            a.set_simulation_robot(on_off=False)
            a.motion_enable(enable=True)
            a.clean_warn()
            a.clean_error()
            a.set_mode(0)
            a.set_state(0)
            a.set_servo_angle(angle=[0.0, 0.0, 0.0, 90, 0.0, 0.0, 0.0], wait=False, speed=20, acceleration=5, is_radian=False)

if __name__ == "__main__":

    from xarm.wrapper import XArmAPI

    ROBOT = "xArms"
    PORT = 5004

    arm1 = XArmAPI('192.168.1.203')
    # arm2 = XArmAPI('192.168.1.242')
    arm3 = XArmAPI('192.168.1.237')
    arm4 = XArmAPI('192.168.1.244')
    arm5 = XArmAPI('192.168.1.234')
    arm6 = XArmAPI('192.168.1.215')
    arm7 = XArmAPI('192.168.1.208')
    arm8 = XArmAPI('192.168.1.236')
    arm9 = XArmAPI('192.168.1.211')

    arms = [arm1, 0, arm3, arm4, arm5, arm6, arm7, arm8, arm9]
    totalArms = len(arms)

    setup()

    repeat = input("do we need to repeat? [y/n]")
    if repeat == 'y':
        setup()
    for a in arms:
        if a != 0:
            a.set_mode(1)
            a.set_state(0)

    graph = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [0.0, 1.0], [1.0, 1.0], [2.0, 1.0], [0.0, 2.0], [1.0, 2.0], [2.0, 2.0]])

    pos_que = []

    for i in range(len(graph)):
        pos_que.append(queue.Queue())

    t_arms = []

    for i in range(len(graph)):
        t_arms.append(Thread(target=playRobot, args=(pos_que[i], arms[i],)))

    deltas = []

    # for i in range(len(graph)):
    #     if i != 1:
    #         t_arms[i].start()

    t_arms[0].start()
    t_arms[2].start()
    t_arms[3].start()
    t_arms[4].start()
    t_arms[5].start()
    t_arms[6].start()
    t_arms[7].start()
    t_arms[8].start()

    while True:
        print("Enter coordinates (floating number)")
        x = float(input("X: "))
        y = float(input("Y: "))
        dancer_pos = [x, y]
        j6 = findPositionAngle(dancer_pos, graph)
        for i in range(len(graph)):
            # curr = arms[i].angles[2]
            pos_que[i].put([0, 0, j6[i], 90, 0, 0, 0])
        print(j6[0])





