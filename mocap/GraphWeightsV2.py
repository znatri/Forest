import math
import random
import numpy as np

def distanceWeights(originbot, otherbots):
    weights = {}

    for i in range(len(otherbots)):
        distance = findDistance(originbot, otherbots[i])

        weight = np.interp(distance, [0, 2], [1, 0.25])

        w1 = int(100 * (weight - 0.2))
        w2 = int(100 * (weight + 0.2))

        weights[i+1] = abs(random.randint(w1, w2) / 100)

    return weights

def findDistance(origin, newpoint):
    distance = ((((newpoint[0] - origin[0]) ** 2) + ((newpoint[1] - origin[1]) ** 2)) ** 0.5)
    return distance

if __name__ == "__main__":
    graph = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [0.0, 1.0], [1.0, 1.0], [2.0, 1.0], [0.0, 2.0], [1.0, 2.0], [2.0, 2.0]])
    num = int(input("Enter an int 1-9:"))
    originbot = graph[num-1]
    weights = distanceWeights(originbot, graph)
    print(weights)

    # {1: 0.89, 2: 1.12, 3: 0.72, 4: 0.35, 5: 0.64, 6: 0.56, 7: 0.32, 8: 0.31, 9: 0.54}
