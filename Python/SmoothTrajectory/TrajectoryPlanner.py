import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.widgets import TextBox, Button
import ForwardKinematics as ForwardK
import time

fig = plt.figure(figsize=(5, 6))
ax = fig.add_subplot(projection='3d')
plt.subplots_adjust(bottom=0.4)
maxZ = 400
minZ = -400
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.view_init(azim=30)

alpha = np.array([0, -np.pi/2, 0, -np.pi/2, -np.pi/2, np.pi/2])
toolPosition = np.array([0, 0, 60])

xOff = np.array([22.12, 0, 300.32, 0, 0, 0])
yOff = np.array([0, 31.8, 0, 293, 0, -62])
zOff = np.array([135.7, 0, 0, -36.3, 0, 0])

trail = np.empty([1,3])
prevEndPosition = np.array([0, 0, 0])  # setup for draw trail
desiredToolPos = np.array([0, 0, 0])
desiredToolOri = np.array([0, 0, 0])

axDrawButton = plt.axes([0.2, 0.22, 0.2, 0.04])
changeN = plt.axes([0.2, 0.16, 0.2, 0.04])
xyzInput = plt.axes([0.2, 0.1, 0.55, 0.04])
abgInput = plt.axes([0.2, 0.04, 0.55, 0.04])

drawTrajectory = Button(axDrawButton, 'Start')
changeNText = TextBox(changeN, 'N: ')
xyzInputText = TextBox(xyzInput, 'Desired Pos: ')
abgInputText = TextBox(abgInput, 'Desired Ori: ')

r1_1 = 22.12
r2_1 = 135.7
r3_1 = 31.8
d1_1 = 300.32
d3_1 = -36.3
d4_1 = 293
d6_1 = 62
endEffector = [60, 0, 0]

xOff = np.array([r1_1, 0, 0, d4_1, 0, d6_1])
yOff = np.array([0, r3_1, 0, d3_1, 0, 0])
zOff = np.array([r2_1, 0, d1_1, 0, 0, 0])

def getToolPositionDistance(thetas, desiredToolPos):
		currentToolPos = ForwardK.getEndEffectorData(thetas)[0]
		currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)
		return currentToolPos, currentDistance

def findEndThetas():
	global thetas
	np.set_printoptions(precision = 2)
	# draw desired end effector location
	xEnd = desiredToolPos[0]
	yEnd = desiredToolPos[1]
	zEnd = desiredToolPos[2]
	alpha = desiredToolOri[0]
	beta = desiredToolOri[1]
	gamma = desiredToolOri[2]
	
	# alpha, beta, gamma input as degrees
 
	xEnd = float(xEnd)
	yEnd = float(yEnd)
	zEnd = float(zEnd)
	alpha = float(alpha)
	beta = float(beta)
	gamma = float(gamma)

	# convert to radians
	alpha = alpha * np.pi / 180
	beta = beta * np.pi / 180
	gamma = gamma * np.pi / 180
 
	lengths = np.array([135.7, 300.32, 293])
 
	# calculate position of spherical wrist
	transform0End = np.array([[np.cos(beta)*np.cos(gamma), -np.cos(beta)*np.sin(gamma), np.sin(beta), xEnd],
							  [np.cos(alpha)*np.sin(gamma) + np.cos(gamma)*np.sin(alpha)*np.sin(beta), np.cos(alpha)*np.cos(gamma) - np.sin(alpha)*np.sin(beta)*np.sin(gamma), -np.cos(beta)*np.sin(alpha), yEnd],
							  [-np.cos(alpha)*np.cos(gamma)*np.sin(beta) + np.sin(alpha)*np.sin(gamma), np.cos(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(gamma)*np.sin(alpha), np.cos(alpha)*np.cos(beta), zEnd],
							  [0, 0, 0, 1]])
	transformEndSw = np.array([[1, 0, 0, -d6_1-60],
							   [0, 1, 0, 0],
							   [0, 0, 1, 0],
							   [0, 0, 0, 1]])
	transform0Sw = transform0End @ transformEndSw

	sphericalPos = np.vstack(transform0Sw[:-1,3])
	sphericalPos = np.array(sphericalPos).ravel()

	print(f'Spherical pos: {sphericalPos}')

	xSw = sphericalPos[0]
	ySw = sphericalPos[1]
	zSw = sphericalPos[2]
 
	hSw = zSw - lengths[0]
 
	if np.sqrt(xSw**2 + ySw**2 + hSw**2) > sum(lengths[1:]):
		print("Desired position and orientation not in workspace.")
	else:
		RSw = np.sqrt(xSw**2 + ySw**2)
		rSw = np.sqrt(hSw**2 + RSw**2)
		alpha2 = np.arcsin(hSw/rSw)

		# First three joint angles responsible for placing end effector
		# Using law of cosines:
		theta1 = np.arctan2(ySw, xSw)
		theta2 = -np.arccos((lengths[1]**2 - lengths[2]**2 + rSw**2)/(2*lengths[1]*rSw)) + np.pi/2 - alpha2
		theta3 = -np.arccos((lengths[2]**2 + lengths[1]**2 - rSw**2)/(2*lengths[1]*lengths[2])) + np.pi/2

		
		# remove list att from th 1 2 and 3
		theta1 = float(theta1)
		theta2 = float(theta2)
		theta3 = float(theta3)


		# Final three joint angles specify rotation only

		# rot_mat_0_6_1 = np.array([[np.cos(beta)*np.cos(gamma), 												-np.cos(beta)*np.sin(gamma), 											np.sin(beta)], 
		# 						[np.cos(alpha)*np.sin(gamma) + np.cos(gamma)*np.sin(alpha)*np.sin(beta), 	np.cos(alpha)*np.cos(gamma) - np.sin(alpha)*np.sin(beta)*np.sin(gamma), -np.cos(beta)*np.sin(alpha)], 
		# 						[-np.cos(alpha)*np.cos(gamma)*np.sin(beta) + np.sin(alpha)*np.sin(gamma), 	np.cos(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(gamma)*np.sin(alpha), np.cos(alpha)*np.cos(beta)]])
		rot_mat_0_6 = transform0End[:3][:3]
		# print(rot_mat_0_6_1)
		# print(rot_mat_0_6)
		rot_mat_0_3 = np.array([[np.cos(theta1)*np.cos(theta2)*np.cos(theta3) - np.cos(theta1)*np.sin(theta2)*np.sin(theta3), 	-np.sin(theta1), 	np.cos(theta1)*np.cos(theta2)*np.sin(theta3) + np.cos(theta1)*np.cos(theta3)*np.sin(theta2)], 
								[np.cos(theta2)*np.cos(theta3)*np.sin(theta1) - np.sin(theta1)*np.sin(theta2)*np.sin(theta3), 	np.cos(theta1), 	np.cos(theta2)*np.sin(theta1)*np.sin(theta3) + np.cos(theta3)*np.sin(theta1)*np.sin(theta2)], 
								[-np.cos(theta2)*np.sin(theta3) - np.cos(theta3)*np.sin(theta2), 								0, 					np.cos(theta2)*np.cos(theta3) - np.sin(theta2)*np.sin(theta3)]])

		inv_rot_mat_0_3 = np.linalg.inv(rot_mat_0_3)
		# rot_mat_3_6_1 = inv_rot_mat_0_3 @ rot_mat_0_6_1
		# print(rot_mat_3_6_1)
		rot_mat_3_6 = inv_rot_mat_0_3 @ rot_mat_0_6
		# print(rot_mat_3_6)
		

		theta5 = np.arccos(rot_mat_3_6[0,0])
		
		theta6_1 = np.arcsin(rot_mat_3_6[0,1]/np.sin(theta5))
		theta6_2 = np.pi - np.arcsin(rot_mat_3_6[0,1]/np.sin(theta5))

		theta4_1 = np.arcsin(rot_mat_3_6[1,0]/np.sin(theta5))
		theta4_2 = np.pi - np.arcsin(rot_mat_3_6[1,0]/np.sin(theta5))
		

		# calculate tool positions for both possible angles to see which angle produces a closer result
		transform01 = np.array([[np.cos(theta1), -np.sin(theta1), 0, xOff[0]], 
								[np.sin(theta1), np.cos(theta1), 0, yOff[0]], 
								[0, 0, 1, zOff[0]],
								[0, 0, 0, 1]])
		transform12 = np.array([[np.cos(theta2), 0, np.sin(theta2), xOff[1]],
								[0, 1, 0, yOff[1]],
								[-np.sin(theta2), 0, np.cos(theta2), zOff[1]], 
								[0, 0, 0, 1]])
		transform23 = np.array([[np.cos(theta3), 0, np.sin(theta3), xOff[2]],
								[0, 1, 0, yOff[2]],
								[-np.sin(theta3), 0, np.cos(theta3), zOff[2]],
								[0, 0, 0, 1]])
		transform34_1 = np.array([[1, 0, 0, xOff[3]],
								[0, np.cos(theta4_1), -np.sin(theta4_1), yOff[3]],
								[0, np.sin(theta4_1), np.cos(theta4_1), zOff[3]],
								[0, 0, 0, 1]])
		transform45 = np.array([[np.cos(theta5), 0, np.sin(theta5), xOff[4]], 
								[0, 1, 0, yOff[4]], 
								[-np.sin(theta5), 0, np.cos(theta5), zOff[4]], 
								[0, 0, 0, 1]])
		transform56_1 = np.array([[1, 0, 0, xOff[5]], 
								[0, np.cos(theta6_1), -np.sin(theta6_1), yOff[5]], 
								[0, np.sin(theta6_1), np.cos(theta6_1), zOff[5]], 
								[0, 0, 0, 1]])

		transform34_2 = np.array([[1, 0, 0, xOff[3]],
								[0, np.cos(theta4_2), -np.sin(theta4_2), yOff[3]],
								[0, np.sin(theta4_2), np.cos(theta4_2), zOff[3]],
								[0, 0, 0, 1]])
		transform56_2 = np.array([[1, 0, 0, xOff[5]], 
								[0, np.cos(theta6_2), -np.sin(theta6_2), yOff[5]], 
								[0, np.sin(theta6_2), np.cos(theta6_2), zOff[5]], 
								[0, 0, 0, 1]])
		# Working position of tool in end effector coordinates
		transform6Tool = np.array([[1, 0, 0, endEffector[0]],
								[0, 1, 0, endEffector[1]],
								[0, 0, 1, endEffector[2]],
								[0, 0, 0, 1]])

		transform = np.array([transform01, transform12, transform23, transform34_1, transform45, transform56_1, transform34_2, transform56_2, transform6Tool])
		
		transform02 = transform[0] @ transform[1]
		transform03 = transform[0] @ transform[1] @ transform[2]
		transform04_1 = transform[0] @ transform[1] @ transform[2] @ transform[3]
		transform05_1 = transform[0] @ transform[1] @ transform[2] @ transform[3] @ transform[4]
		transform06_1 = transform[0] @ transform[1] @ transform[2] @ transform[3] @ transform[4] @ transform[5]
		toolPos06_1 = transform06_1 @ transform6Tool
		transform04_2 = transform[0] @ transform[1] @ transform[2] @ transform[6]
		transform05_2 = transform[0] @ transform[1] @ transform[2] @ transform[6] @ transform[4]
		transform06_2 = transform[0] @ transform[1] @ transform[2] @ transform[6] @ transform[4] @ transform[7]
		toolPos06_2 = transform06_2 @ transform6Tool

		baseTransforms1 = np.array([transform01, transform02, transform03, transform04_1, transform05_1, transform06_1, toolPos06_1])
		baseTransforms2 = np.array([transform01, transform02, transform03, transform04_2, transform05_2, transform06_2, toolPos06_2])
		toolpos1 = toolPos06_1[:-1,3]
		toolpos2 = toolPos06_2[:-1,3]

		distFrom1 = np.sqrt((toolpos1[0] - xEnd)**2 + (toolpos1[1] - yEnd)**2 + (toolpos1[2] - zEnd)**2)
		distFrom2 = np.sqrt((toolpos2[0] - xEnd)**2 + (toolpos2[1] - yEnd)**2 + (toolpos2[2] - zEnd)**2)

		theta4 = 0
		theta6 = 0
		# choose theta 4 and 6 pair that results in least distance between desired and calculated end effector position
		if distFrom1 < distFrom2:
			theta4 = theta4_1
			theta6 = theta6_1
		else:
			theta4 = theta4_2
			theta6 = theta6_2


		# print(f'\nxSw: {xSw}, : {ySw}, zSw: {zSw}')
		# print(f'\nAngles in radians:\ntheta1: {theta1}\ntheta2: {theta2}\ntheta3: {theta3}\ntheta4: {theta4}\ntheta5: {theta5}\ntheta6: {theta6}')
		# print(f'\nAngles in degrees:\ntheta1: {theta1*180/np.pi}\ntheta2: {theta2*180/np.pi}\ntheta3: {theta3*180/np.pi}\ntheta4: {theta4*180/np.pi}\ntheta5: {theta5*180/np.pi}\ntheta6: {theta6*180/np.pi}')
  
		thetas = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
		thetas = thetas * 180/np.pi
  
		#recalculate forward kinematics to compare
		jointPos, jointRot, _ = ForwardK.updateMatrices(thetas)
		# print(f"jointPos[5]: {jointPos[5]}")
		# print(f"jointPos[6]: {jointPos[6]}")
		# print(f"jointRot[5]: {jointRot[5]}")		# todo: otolpos and toolrot don't match with original values
		# print(f"jointRot[6]: {jointRot[6]}")
		# print(f"\ntransform0End: {transform0End}")
		print(f"\nxError: {xEnd - jointPos[6][0]}")
		print(f"yError: {yEnd - jointPos[6][1]}")
		print(f"zError: {zEnd - jointPos[6][2]}")
		# currentJointPositions, baseTransforms = ForwardK.updateMatrices(thetas)[0::2]
		# # print("JOINT POSITIONS: {}".format(currentJointPositions))
		# drawArm(currentJointPositions, baseTransforms)
	
	return thetas


plt.style.use('seaborn-whitegrid')

#joint trajectory values
Tf = 4  # how long the motion should take
N = 20  # number of steps in motion
method = 3  # method to determine path (3 or 5) (cubic or quintic)
# declare initial and final joint values
thetastart = np.array([2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi])
thetaend = np.zeros(6)
thetas = np.array(thetastart).copy()

def drawEverything(jointPositions):
	ax.clear()
	ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)
	plotPointsLines(jointPositions)
	drawTrail(jointPositions)
	ax.axis([-400, 400, -400, 400])
	ax.set_xlabel("x")
	ax.set_ylabel("y")
	ax.set_zlabel("z")
	fig.canvas.draw_idle()

def plotPointsLines(jointPositions):
	jointPositions = np.insert(jointPositions, 0, [0, 0, 0], axis=0)
	# Plot points
	ax.scatter(jointPositions[:,0], jointPositions[:,1], jointPositions[:,2], color='.3')
	# Plot lines
	xlines = np.array(jointPositions[:,0])
	ylines = np.array(jointPositions[:,1])
	zlines = np.array(jointPositions[:,2])
	ax.plot3D(xlines, ylines, zlines, 'chartreuse')

def drawTrail(jointPositions):
	global prevEndPosition
	global trail
	currentEndPosition = jointPositions[len(jointPositions)-1]
	diffx = int(abs(prevEndPosition[0] - currentEndPosition[0]))
	diffy = int(abs(prevEndPosition[1] - currentEndPosition[1]))
	diffz = int(abs(prevEndPosition[2] - currentEndPosition[2]))
	if diffx != 0 and diffy != 0 and diffz != 0:
		trail = np.append(trail, [currentEndPosition], axis=0)
		prevEndPosition = currentEndPosition
	ax.scatter(trail[:,0], trail[:,1], trail[:,2], s=10, color="orange")
	for i in range(1, len(trail)):
		ax.text(trail[i,0], trail[i,1], trail[i,2], i,fontsize=10, color='red')
	prevEndPosition = currentEndPosition
initialJointPositions = ForwardK.updateMatrices(thetastart)[0]
print(initialJointPositions)
drawEverything(initialJointPositions)


def start(text):
	global thetastart
	global thetaend
	global trail
	thetaend = findEndThetas()
	traj = mr.JointTrajectory(thetastart, thetaend, Tf, N, method)
	jointPositionMat = np.array([ForwardK.updateMatrices(traj[i])[0] for i in range(N)])
	for i in range(N):
		drawEverything(jointPositionMat[i])
		print(jointPositionMat[i])
		print("pos {}".format(i+1))T
	# reset everything TODO fix errors this might be causing
	thetastart = thetaend
	thetaend = np.zeros(6)
	trail = np.empty([1,3])

def changeNum(text):
	global N
	N = int(text)

def changeGoalPosition(text):
	global desiredToolPos
	np.set_printoptions(precision = 2)
	x, y, z = text.split(" ")
	print("x: {}, y: {}, z: {}".format(x, y, z))
	desiredToolPos = np.array([int(x), int(y), int(z)])

def changeGoalOrientation(text):
	global desiredToolOri
	np.set_printoptions(precision = 2)
	a, b, g = text.split(" ")
	print("a: {}, b: {}, g: {}".format(a, b, g))
	desiredToolOri = np.array([int(a), int(b), int(g)])


xyzInputText.on_submit(changeGoalPosition)
abgInputText.on_submit(changeGoalOrientation)
drawTrajectory.on_clicked(start)
changeNText.on_submit(changeNum)

plt.show()