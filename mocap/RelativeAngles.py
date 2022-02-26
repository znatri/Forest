import numpy as np
import math

def findRelativeAngle(pos, arm_pos):
    (x, y) = pos
    (a, b) = arm_pos

    if (x, y) == (a, b):
        return 0

    opposite = findDistance((x, y), (a, y))
    hypotenuse = findDistance((x, y), (a, b))
    angleRad = math.asin(opposite/hypotenuse)
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

def findDistance(origin, newpoint):
    distance = ((((newpoint[0] - origin[0]) ** 2) + ((newpoint[1] - origin[1]) ** 2)) ** 0.5)
    return distance

if __name__ == "__main__":

    graph = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [0.0, 1.0], [1.0, 1.0], [2.0, 1.0], [0.0, 2.0], [1.0, 2.0], [2.0, 2.0]])
    print("Enter coordinates (floating number)")
    x = float(input("X: "))
    y = float(input("Y: "))
    pos = [x, y]

    for i in range(9):
        angle = findRelativeAngle(pos, graph[i])
        print(f"arm: {i+1}, coordinate {graph[i]}, relative angle: {angle}")
