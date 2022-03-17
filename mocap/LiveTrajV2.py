from threading import Thread
import queue
import numpy as np
import math
import time

def playRobot(que, arm, mapangle):
    tf_pos = 50
    tf_shadow = 5
    t_step = 0.006
    t_array_pos = np.arange(0, tf_pos, t_step)
    t_array_shadow = np.arange(0, tf_shadow, t_step)

    q_dot_f = np.zeros(7)
    q_dotdot_f = np.zeros(7)
    p = [0, 0, 0, 90, 0, 0, 0]

    while True:
        j3 = que.get()
        data = mapangle.get()
        j5 = data.get("j5")
        j6 = data.get("j6")
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
                break

            if j == len(t_array_pos):
                t_pos = tf_pos
            else:
                t_pos = t_array_pos[j]

            if k >= len(t_array_shadow):
                t_shadow = tf_shadow
            else :
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

            # arm.set_servo_angle_j(angles=p, is_radian=False)
            print(f"{p} {arm}")

            tts = time.time() - start_time
            sleep = t_step - tts

            if tts > t_step:
                sleep = 0

            time.sleep(sleep)
            j += 1
            k += 1

if __name__ == "__main__":
    arms = [1, 2, 3, 4, 5, 6, 7, 8, 9]

    pos_que = queue.Queue()
    mapanlge_que = queue.Queue()

    t_arm = Thread(target=playRobot, args=(pos_que, arms[0], mapanlge_que))
    t_arm.start()


    print("Enter coordinates (floating number)")
    j3 = float(input("J3: "))
    j5 = float(input("J5: "))
    j6 = float(input("J6: "))

    pos_que.put(j3)
    mapanlge_que.put(
        {
            'j5': j5,
            'j6': j6,
        }
    )