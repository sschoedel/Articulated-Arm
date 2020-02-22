import numpy as np

segXs = np.array([0, 0, 10, 10])
segYs = np.array([0, 0, 0, 0])
segZs = np.array([0, 10, 0, 0])

def normalize3D(vector):
	return np.sqrt(sum(vector[:]**2))

toolDist = np.array([3, 0, 0])

omegas = np.array([[0, 0, 1],[0, 1, 0],[0, 1, 0],[0, 1, 0]])

initialQs = np.array([[sum(segXs[:i]),sum(segYs[:i]),sum(segZs[:i])] for i in range(1, 5)])

Vs = np.array([np.cross(omegas[i], initialQs[i]) for i in range(len(omegas))])

Ses = np.array([[[omegas[i], Vs[i]]] for i in range(len(omegas))]).reshape((-1,6,1))

print(Vs)
# print(Ses)
# print(Vs)
