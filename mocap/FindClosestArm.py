import numpy as np

def closest_arm(pos, nodes):
    # nodes = np.asarray(nodes)
    deltas = nodes - pos
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    return np.argmin(dist_2)

if __name__ == "__main__":
    graph = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [0.0, 1.0], [1.0, 1.0], [2.0, 1.0], [0.0, 2.0], [1.0, 2.0], [2.0, 2.0]])
    pos = [2.1, 1.2]
    print(closest_arm(pos, graph))