import queue

def bfs(graph, source, dist):
    Q = queue.Queue()
    visited_vertices = set()
    Q.put(source)
    distance = 0
    dist[source] = 0
    visited_vertices.update({0})
    while not Q.empty():
        vertex = Q.get()
        distance += 1
        print(vertex, end="-->")
        for u in graph[vertex]:
            if u not in visited_vertices:
                Q.put(u)
                dist[u] = distance
                visited_vertices.update({u})

def distanceWeights(num):
    # returns dictionary of weights to use for each arm
    graph = {
        1: [2, 4, 5],
        2: [1, 3, 4, 5, 6],
        3: [2, 5, 6],
        4: [1, 2, 5, 7, 8],
        5: [1, 2, 3, 4, 6, 7, 8, 9],
        6: [2, 3, 5, 8, 9],
        7: [4, 5, 8],
        8: [4, 5, 6, 7, 9],
        9: [5, 6, 8]
    }

    dist = {
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

    bfs(graph, num, dist)

    print()
    print(dist)

distanceWeights(1)

