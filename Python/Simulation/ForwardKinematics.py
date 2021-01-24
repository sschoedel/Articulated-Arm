import numpy as np
import matrixHelpers as mh

numRotations = 2

thetas = np.array([360*numRotations/2] * 6)

alpha = np.array([0, -np.pi/2, 0, -np.pi/2, -np.pi/2, np.pi/2])
toolPosition = np.array([60, 0, 0])

# r1 = 47
# r2 = 110
# r3 = 26
# d1 = 133
# d3 = 0
# d4 = 117.5
# d6 = 28

r1_1 = 22.12
r2_1 = 135.7
r3_1 = 31.8
d1_1 = 300.32
d3_1 = 36.3
d4_1 = 293
d6_1 = 62

# for comparing ik code
# r1_1 = 0
# r2_1 = 135.7
# r3_1 = 0
# d1_1 = 300.32
# d3_1 = 0
# d4_1 = 293
# d6_1 = 62

xOff = np.array([r1_1, 0, 0, d4_1, 0, d6_1])
yOff = np.array([0, r3_1, 0, d3_1, 0, 0])
zOff = np.array([r2_1, 0, d1_1, 0, 0, 0])

r1_2 = 22.12
r2_2 = 31.8
r3_2 = 300.32
d1_2 = 135.7
d3_2 = 293
d4_2 = -36.3
d6_2 = 62
 
# xOff = np.array([22.12, 0, 300.32, 0, 0, 0])
# yOff = np.array([0, 31.8, 0, 293, 0, -62])
# zOff = np.array([135.7, 0, 0, -36.3, 0, 0])

def getEndEffectorData(theta):  # more efficient version of update matrice that only returns end effector position array and rotation matrix
	theta = theta * np.pi/180

	toolPosition = np.array([0, 0, 60])
	transform01 = np.array([[np.cos(theta[0]), -np.sin(theta[0]), 0, xOff[0]], 
							[np.sin(theta[0]), np.cos(theta[0]), 0, yOff[0]], 
							[0, 0, 1, zOff[0]], 
							[0, 0, 0, 1]])
	transform12 = np.array([[np.cos(theta[1]), -np.sin(theta[1]), 0, xOff[1]],
							[0, 0, 1, yOff[1]], 
							[-np.sin(theta[1]), -np.cos(theta[1]), 0, zOff[1]], 
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
	print(f'thetas: {theta}')
 
	# Rotation from 0 to 1 = Rx(alpha)Rz(theta)
	# alpha is rotation to next joint location theta varies as arm moves
	# Position is column matrix for translations from the previous frame to the next after rotation
	# T from 0 to 1 = [Rotation matrix, position change]
					# [0,     0,     0,      1         ]
	# P0x = Mult all T01 * T12 * T23 * T34... * T56 * P6x 
	# End effector position to position zero cascades from end effector position in frame six 
	# cascading down from each transition matrix

	# Transformation matrices
	# transform01 = np.array([[np.cos(theta[0]), -np.sin(theta[0]), 0, xOff[0]], 
	# 						[np.sin(theta[0]), np.cos(theta[0]), 0, yOff[0]], 
	# 						[0, 0, 1, zOff[0]], 
	# 						[0, 0, 0, 1]])
	# transform12 = np.array([[np.cos(theta[1]), -np.sin(theta[1]), 0, xOff[1]],
	# 						[0, 0, 1, yOff[1]], 
	# 						[-np.sin(theta[1]), -np.cos(theta[1]), 0, zOff[1]], 
	# 						[0, 0, 0, 1]])
	# transform23 = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0, xOff[2]], 
	# 						[np.sin(theta[2]), np.cos(theta[2]), 0, yOff[2]], 
	# 						[0, 0, 1, zOff[2]], 
	# 						[0, 0, 0, 1]])
	transform01 = np.array([[np.cos(theta[0]), -np.sin(theta[0]), 0, xOff[0]], 
							[np.sin(theta[0]), np.cos(theta[0]), 0, yOff[0]], 
							[0, 0, 1, zOff[0]], 
							[0, 0, 0, 1]])
	transform12 = np.array([[np.cos(theta[1]), 0, np.sin(theta[1]), xOff[1]],
							[0, 1, 0, yOff[1]], 
							[-np.sin(theta[1]), 0, np.cos(theta[1]), zOff[1]], 
							[0, 0, 0, 1]])
	transform23 = np.array([[np.cos(theta[2]), 0, np.sin(theta[2]), xOff[2]],
							[0, 1, 0, yOff[2]], 
							[-np.sin(theta[2]), 0, np.cos(theta[2]), zOff[2]], 
							[0, 0, 0, 1]])
	# transform34 = np.array([[np.cos(theta[3]), -np.sin(theta[3]), 0, xOff[3]], 
	# 						[0, 0, 1, yOff[3]],
	# 						[-np.sin(theta[3]), -np.cos(theta[3]), 0, zOff[3]], 
	# 						[0, 0, 0, 1]])
	# transform45 = np.array([[np.cos(theta[4]), -np.sin(theta[4]), 0, xOff[4]], 
	# 						[0, 0, 1, yOff[4]],
	# 						[-np.sin(theta[4]), -np.cos(theta[4]), 0, zOff[4]], 
	# 						[0, 0, 0, 1]])
	# transform56 = np.array([[np.cos(theta[5]), -np.sin(theta[5]), 0, xOff[5]], 
	# 						[0, 0, -1, yOff[5]],
	# 						[np.sin(theta[5]), np.cos(theta[5]), 0, zOff[5]], 
	# 						[0, 0, 0, 1]])
	transform34 = np.array([[1, 0, 0, xOff[3]], 
							[0, np.cos(theta[3]), -np.sin(theta[3]), yOff[3]], 
							[0, np.sin(theta[3]), np.cos(theta[3]), zOff[3]], 
							[0, 0, 0, 1]])
	transform45 = np.array([[np.cos(theta[4]), 0, np.sin(theta[4]), xOff[4]], 
							[0, 1, 0, yOff[4]], 
							[-np.sin(theta[4]), 0, np.cos(theta[4]), zOff[4]], 
							[0, 0, 0, 1]])
	transform56 = np.array([[1, 0, 0, xOff[5]], 
							[0, np.cos(theta[5]), -np.sin(theta[5]), yOff[5]], 
							[0, np.sin(theta[5]), np.cos(theta[5]), zOff[5]], 
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

def ForwardK(theta):
	# forward kinematics
	# input: Jfk - joints value for the calculation of the forward kinematics
	# output: Xfk - pos value for the calculation of the forward kinematics

	r = np.array([r1_2, r2_2, r3_2, d3_2, 0.0, d6_2])
	d = np.array([d1_2, 0.0, 0.0, d4_2, 0.0, 0.0])

	# Denavit-Hartenberg matrix
	theTemp = np.array([0.0, 90.0, 0.0, 90.0, 0.0, -90.0])
	theta = np.add(theTemp, theta)
	alfa = np.array([-90.0, 0.0, -90.0, 90.0, -90.0, 0.0])
	# r = np.array([r1_2, r2_2, r3_2, 0.0, 0.0, 0.0])
	# d = np.array([d1_2, 0.0, d3_2, d4_2, 0.0, d6_2])
	# from deg to rad
	theta = theta * np.pi/180
	alfa = alfa * np.pi/180

	# work frame
	Xwf = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Xwf=[0 0 0 0 0 0]

	# tool frame
	Xtf = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Xtf=[0 0 0 0 0 0]

	# work frame transformation matrix
	Twf = mh.pos2tran(Xwf) # Twf=mh.pos2tran(Xwf)

	# tool frame transformation matrix
	Ttf = mh.pos2tran(Xtf) # Ttf=mh.pos2tran(Xtf)

	# DH homogeneous transformation matrix
	T01 = mh.DH1line(theta[0], alfa[0], r[0], d[0]) # T01=mh.DH1line(theta(1),alfa(1),r(1),d(1))
	T12 = mh.DH1line(theta[1], alfa[1], r[1], d[1]) # T12=mh.DH1line(theta(2),alfa(2),r(2),d(2))
	T23 = mh.DH1line(theta[2], alfa[2], r[2], d[2]) # T23=mh.DH1line(theta(3),alfa(3),r(3),d(3))
	T34 = mh.DH1line(theta[3], alfa[3], r[3], d[3]) # T34=mh.DH1line(theta(4),alfa(4),r(4),d(4))
	T45 = mh.DH1line(theta[4], alfa[4], r[4], d[4]) # T45=mh.DH1line(theta(5),alfa(5),r(5),d(5))
	T56 = mh.DH1line(theta[5], alfa[5], r[5], d[5]) # T56=mh.DH1line(theta(6),alfa(6),r(6),d(6))

	Tw1 = Twf @ T01
	Tw2 = Tw1 @ T12
	Tw3 = Tw2 @ T23
	Tw4 = Tw3 @ T34
	Tw5 = Tw4 @ T45
	Tw6 = Tw5 @ T56
	Twt = Tw6 @ Ttf

	# calculate pos from transformation matrix
	Xfk = mh.tran2pos(Twt) # Xfk=mh.tran2pos(Twt)
	# Xfk(4:6)=Xfk(4:6)/np.pi*180
	Xfk[3] = Xfk[3]/np.pi*180.0
	Xfk[4] = Xfk[4]/np.pi*180.0
	Xfk[5] = Xfk[5]/np.pi*180.0
 
	baseTransforms = np.array([Tw1, Tw2, Tw3, Tw4, Tw5, Tw6, Twt])

	# Extract position and rotation data for each joint
	jointPositions = np.array([baseTransforms[i,:-1,3] for i in range(0, baseTransforms.shape[0])])
	jointRotationMatrices = np.array([baseTransforms[i,:-1,:-1] for i in range(0, baseTransforms.shape[0])])
	return jointPositions, jointRotationMatrices, baseTransforms

print(updateMatrices(np.array([0,0,0,0,0,0]))[0])

if __name__ == '__main__':
	pass