import math
import random
import numpy as np
import time

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

def findDistances(pos, nodes):
    deltas = nodes - pos
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    dist = np.sqrt(dist_2)
    return dist

if __name__ == "__main__":
    graph = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [0.0, 1.0], [1.0, 1.0], [2.0, 1.0], [0.0, 2.0], [1.0, 2.0], [2.0, 2.0]])
    num = int(input("Enter an int 1-9:"))
    leader = graph[num-1]
    weights = findWeights(leader, graph)
    print(weights)