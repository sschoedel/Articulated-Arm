import numpy as np

numRotations = 2

thetas = np.array([360*numRotations/2] * 6)

alpha = np.array([0, -np.pi/2, 0, -np.pi/2, -np.pi/2, np.pi/2])
toolPosition = np.array([0, 0, 60])

xOff = np.array([22.12, 0, 300.32, 0, 0, 0])
yOff = np.array([0, 31.8, 0, 293, 0, -62])
zOff = np.array([135.7, 0, 0, -36.3, 0, 0])

def getEndEffectorData(theta):  # more efficient version of update matrice that only returns end effector position array and rotation matrix
	theta = theta * np.pi/180

	toolPosition = np.array([0, 0, 60])
	transform01 = np.array([[np.cos(theta[0]), -np.sin(theta[0]), 0, xOff[0]], 
							[np.sin(theta[0]), np.cos(theta[0]), 0, yOff[0]], 
							[0, 0, 1, zOff[0]], 
							[0, 0, 0, 1]])
	transform12 = np.array([[np.cos(theta[1] - np.pi/2), -np.sin(theta[1] - np.pi/2), 0, xOff[1]],
							[0, 0, 1, yOff[1]], 
							[-np.sin(theta[1] - np.pi/2), -np.cos(theta[1] - np.pi/2), 0, zOff[1]], 
							[0, 0, 0, 1]])
	transform23 = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0, xOff[2]], 
							[np.sin(theta[2]), np.cos(theta[2]), 0, yOff[2]], 
							[0, 0, 1, zOff[2]], 
							[0, 0, 0, 1]])
	transform34 = np.array([[np.cos(theta[3]), -np.sin(theta[3]), 0, xOff[3]], 
							[0, 0, 1, yOff[3]], 
							[-np.sin(theta[3]), -np.cos(theta[3]), 0, zOff[3]], 
							[0, 0, 0, 1]])
	transform45 = np.array([[np.cos(theta[4]), -np.sin(theta[4]), 0, xOff[4]], 
							[0, 0, 1, yOff[4]], 
							[-np.sin(theta[4]), -np.cos(theta[4]), 0, zOff[4]], 
							[0, 0, 0, 1]])
	transform56 = np.array([[np.cos(theta[5]), -np.sin(theta[5]), 0, xOff[5]], 
							[0, 0, -1, yOff[5]], 
							[np.sin(theta[5]), np.cos(theta[5]), 0, zOff[5]], 
							[0, 0, 0, 1]])
	# Working position of tool in end effector coordinates
	transform6Tool = np.array([[1, 0, 0, toolPosition[0]],
							   [0, 1, 0, toolPosition[1]],
							   [0, 0, 1, toolPosition[2]],
							   [0, 0, 0, 1]])

	transform0Tool = transform01 @ transform12 @ transform23 @ transform34 @ transform45 @ transform56 @ transform6Tool

	# Extract position and rotation data for each joint
	toolPosition = np.array(transform0Tool[:-1,3])
	toolRotation = np.array(transform0Tool[:-1,:-1])
	return toolPosition, toolRotation

def updateMatrices(theta):
	theta = theta * np.pi/180

	# Rotation from 0 to 1 = Rx(alpha)Rz(theta)
	# alpha is rotation to next joint location theta varies as arm moves
	# Position is column matrix for translations from the previous frame to the next after rotation
	# T from 0 to 1 = [Rotation matrix, position change]
					# [0,     0,     0,      1         ]
	# P0x = Mult all T01 * T12 * T23 * T34... * T56 * P6x 
	# End effector position to position zero cascades from end effector position in frame six 
	# cascading down from each transition matrix

	# Translation matrices
	transform01 = np.array([[np.cos(theta[0]), -np.sin(theta[0]), 0, xOff[0]], 
							[np.sin(theta[0]), np.cos(theta[0]), 0, yOff[0]], 
							[0, 0, 1, zOff[0]], 
							[0, 0, 0, 1]])
	transform12 = np.array([[np.cos(theta[1] - np.pi/2), -np.sin(theta[1] - np.pi/2), 0, xOff[1]],
							[0, 0, 1, yOff[1]], 
							[-np.sin(theta[1] - np.pi/2), -np.cos(theta[1] - np.pi/2), 0, zOff[1]], 
							[0, 0, 0, 1]])
	transform23 = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0, xOff[2]], 
							[np.sin(theta[2]), np.cos(theta[2]), 0, yOff[2]], 
							[0, 0, 1, zOff[2]], 
							[0, 0, 0, 1]])
	transform34 = np.array([[np.cos(theta[3]), -np.sin(theta[3]), 0, xOff[3]], 
							[0, 0, 1, yOff[3]], 
							[-np.sin(theta[3]), -np.cos(theta[3]), 0, zOff[3]], 
							[0, 0, 0, 1]])
	transform45 = np.array([[np.cos(theta[4]), -np.sin(theta[4]), 0, xOff[4]], 
							[0, 0, 1, yOff[4]], 
							[-np.sin(theta[4]), -np.cos(theta[4]), 0, zOff[4]], 
							[0, 0, 0, 1]])
	transform56 = np.array([[np.cos(theta[5]), -np.sin(theta[5]), 0, xOff[5]], 
							[0, 0, -1, yOff[5]], 
							[np.sin(theta[5]), np.cos(theta[5]), 0, zOff[5]], 
							[0, 0, 0, 1]])
	# Working position of tool in end effector coordinates
	transform6Tool = np.array([[1, 0, 0, toolPosition[0]],
							   [0, 1, 0, toolPosition[1]],
							   [0, 0, 1, toolPosition[2]],
							   [0, 0, 0, 1]])

	transform = np.array([transform01, transform12, transform23, transform34, transform45, transform56, transform6Tool])

	# Mult all matrices together to get transformation matrix from frame 0 to frame x
	transform02 = transform[0] @ transform[1]
	transform03 = transform[0] @ transform[1] @ transform[2]
	transform04 = transform[0] @ transform[1] @ transform[2] @ transform[3]
	transform05 = transform[0] @ transform[1] @ transform[2] @ transform[3] @ transform[4]
	transform06 = transform[0] @ transform[1] @ transform[2] @ transform[3] @ transform[4] @ transform[5]
	toolPos06 = transform06 @ transform6Tool

	baseTransforms = np.array([transform01, transform02, transform03, transform04, transform05, transform06, toolPos06])

	# Extract position and rotation data for each joint
	jointPositions = np.array([baseTransforms[i,:-1,3] for i in range(0, baseTransforms.shape[0])])
	jointRotationMatrices = np.array([baseTransforms[i,:-1,:-1] for i in range(0, baseTransforms.shape[0])])
	return jointPositions, jointRotationMatrices, baseTransforms

if __name__ == '__main__':
	pass