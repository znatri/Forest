import queue
import random
import numpy as np

def bfs(graph, start):
    discoveryMap = {
        1: -1,
        2: -1,
        3: -1,
        4: -1,
        5: -1,
        6: -1,
        7: -1,
        8: -1,
        9: -1
    }

    Q = queue.Queue()
    visited = set()

    Q.put(start)
    visited.add(start)

    while not Q.empty():
        v = Q.get()
        for u in graph[v]:
            if u not in visited:
                Q.put(u)
                discoveryMap[u] = v
                visited.add(u)

    return discoveryMap

def distanceWeights(arm):
    weights = {}

    # neighbor_graph = {
    #     1: [2, 4, 5],
    #     2: [1, 3, 4, 5, 6],
    #     3: [2, 5, 6],
    #     4: [1, 2, 5, 7, 8],
    #     5: [1, 2, 3, 4, 6, 7, 8, 9],
    #     6: [2, 3, 5, 8, 9],
    #     7: [4, 5, 8],
    #     8: [4, 5, 6, 7, 9],
    #     9: [5, 6, 8]
    # }

    neighbor_graph = {
        1: [2, 4],
        2: [1, 3, 5],
        3: [2, 6],
        4: [1, 5, 7],
        5: [2, 4, 6, 8],
        6: [3, 5, 9],
        7: [4, 8],
        8: [5, 7, 9],
        9: [6, 8]
    }

    discovery = bfs(neighbor_graph, arm)

    for i in discovery.keys():
        distance = -1
        j = i
        while j != -1:
            j = discovery.get(j)
            distance += 1

        weight = np.interp(distance, [0, 2], [1, 0.5])

        w1 = int(100*(weight - 0.2))
        w2 = int(100*(weight + 0.2))

        weights[i] = abs(random.randint(w1, w2)/100)

    return weights

for x in range(0, 9):
    print(distanceWeights(x+1))
