import numpy as np
import queue
from threading import Thread
import time
# from xarm.wrapper import XArmAPI


def setup():
    for a in arms:
        a.set_simulation_robot(on_off=False)
        # a.motion_enable(enable=True)
        a.clean_warn()
        a.clean_error()
        a.set_mode(0)
        a.set_state(0)
        a.set_servo_angle(angle=[0.0, 0.0, 0.0, 1.57, 0.0, 0, 0.0], wait=False, speed=0.4, acceleration=0.25, is_radian=True)


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


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # arm6 = XArmAPI('192.168.1.211')
    # arms = [arm6]
    # setup()
    # repeat = input("do we need to repeat? [y/n]")
    # if repeat == 'y':
    #     setup()
    # for a in arms:
    #     a.set_mode(1)
    #     a.set_state(0)


    def changeDir():
        while True:
            pos = list(map(int, input("\nEnter the positions for joints 1-7 : ").strip().split()))[:7]
            que.put(pos)
        # que.put([30, 30, 0, 90, 0, 45, 0])
        # que.put([0, 30, 0, 90, 0, 45, 90])


    que = queue.Queue()
    t1 = Thread(target=playRobot, args=(que, 'arm'))
    t2 = Thread(target=changeDir)
    t1.start()
    t2.start()
