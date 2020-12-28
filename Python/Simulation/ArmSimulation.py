import numpy as np
import math
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.widgets import Slider, Button, TextBox
import ForwardKinematics as ForwardK
import matrixHelpers as mh

plt.style.use('seaborn-whitegrid')

numRotations = 2


# Define robot geometry
# r1 = 47
# r2 = 110
# r3 = 26
# d1 = 133
# d3 = 0
# d4 = 117.5
# d6 = 28

# r1 = 22.12
# r2 = 135.7
# r3 = 31.8
# d1 = 300.32
# d3 = -36.3
# d4 = 293
# d6 = -62

r1 = 0
r2 = 135.7
r3 = 0
d1 = 300.32
d3 = 0
d4 = 293
d6 = -62

# Initial theta inputs in degrees
thetas = np.array([360*numRotations/2] * 6)

class Vector(object):
	def __init__(self, xEnd, yEnd, zEnd, xBegin, yBegin, zBegin, length):
		self.distance = length
		self.xDir = (xEnd - xBegin)/self.distance
		self.yDir = (yEnd - yBegin)/self.distance
		self.zDir = (zEnd - zBegin)/self.distance
	def get_direction(self):
		return [self.xDir, self.yDir, self.zDir]

def getRotations(jointRotationMatrices):
	# equations for y x z rotations from rotation matrix
	# beta = np.arctan2(-r31, +-np.sqrt(r11** + r21**))
	# alpha = np.arctan2(r21/cos(beta), r11/cos(beta))
	# gamma = np.arctan2(r32/cos(beta), r33/cos(beta))

	# Get rotation angles of joints and tool from 3x3 rotation matrix produced by multiplying successive matrices
	betaRads = np.vstack([np.arctan2(-jointRotationMatrices[i,2,0], np.sqrt(jointRotationMatrices[i,0,0]**2 + jointRotationMatrices[i,1,0]**2)) for i in range(0, jointRotationMatrices.shape[0])])  # y rotation
	alphaRads = np.vstack([np.arctan2(jointRotationMatrices[i,1,0]/np.cos(betaRads[i]), jointRotationMatrices[i,0,0]/np.cos(betaRads[i])) for i in range(0, jointRotationMatrices.shape[0])])  # x rotation
	gammaRads = np.vstack([np.arctan2(jointRotationMatrices[i,2,1]/np.cos(betaRads[i]), jointRotationMatrices[i,2,2]/np.cos(betaRads[i])) for i in range(0, jointRotationMatrices.shape[0])])  # z rotation
	rotationsRad = np.array([alphaRads, betaRads, gammaRads]).reshape((-1, 3))
	return rotationsRad, rotationsRad * 180/np.pi  # In radians and degrees

def printMatrices(jointPositions, jointAngles):
	print("joint positions")
	print(jointPositions)
	print("")

	print("joint angles")
	print(jointAngles)

def drawLabels(jointPositions):
	jointPositions = np.insert(jointPositions, 0, [0, 0, 0], axis=0)
	for i in range(0, 7):
		ax.text(jointPositions[i,0], jointPositions[i,1], jointPositions[i,2], i, color='orange')
	ax.text(jointPositions[7,0], jointPositions[7,1], jointPositions[7,2], "tool", color='orange')

def plotPointsLines(jointPositions):
	jointPositions = np.insert(jointPositions, 0, [0, 0, 0], axis=0)
	# Plot points
	ax.scatter(jointPositions[:,0], jointPositions[:,1], jointPositions[:,2], color='.3')
	# Plot lines
	xlines = np.array(jointPositions[:,0])
	ylines = np.array(jointPositions[:,1])
	zlines = np.array(jointPositions[:,2])
	ax.plot3D(xlines, ylines, zlines, 'chartreuse')

def plotAxes(jointPositions, baseTransforms):
	axesXYZsGlobal = np.array([[50, 0, 0],[0, 50, 0],[0, 0, 50]])  # global xyz coordinates
	axesXYZsGlobalMatrix = np.array([[[1, 0, 0, axesXYZsGlobal[i,0]],  # make matrix out of global xyz coords
						  	 		  [0, 1, 0, axesXYZsGlobal[i,1]],
						  			  [0, 0, 1, axesXYZsGlobal[i,2]],
						  	 		  [0, 0, 0, 1]] for i in range(0, axesXYZsGlobal.shape[0])])
	# print(axesXYZsGlobalMatrix)
	axesXYZsLocalMatrix = [baseTransforms[t] @ axesXYZsGlobalMatrix[i]  for t in range(0, baseTransforms.shape[0]) for i in range(0, axesXYZsGlobal.shape[0])]  # transform into each joint's coords
	axesXYZsLocalMatrix = np.reshape(axesXYZsLocalMatrix, (21, 4, -1))  # reshape for usability

	axesXYZsLocal = np.array([axesXYZsLocalMatrix[i,:-1,3] for i in range(0, axesXYZsLocalMatrix.shape[0])])  # extract position data from matrix

	localXAxes = axesXYZsLocal[0::3]
	localYAxes = axesXYZsLocal[1::3]
	localZAxes = axesXYZsLocal[2::3]

	[ax.plot3D((jointPositions[i,0], localXAxes[i,0]),(jointPositions[i,1], localXAxes[i,1]),(jointPositions[i,2], localXAxes[i,2]), 'red') for i in range(0,7)]
	[ax.plot3D((jointPositions[i,0], localYAxes[i,0]),(jointPositions[i,1], localYAxes[i,1]),(jointPositions[i,2], localYAxes[i,2]), 'green') for i in range(0,7)]
	[ax.plot3D((jointPositions[i,0], localZAxes[i,0]),(jointPositions[i,1], localZAxes[i,1]),(jointPositions[i,2], localZAxes[i,2]), 'blue') for i in range(0,7)]


# functions for widgets
def reset(event):
	sliderTheta1.reset()
	sliderTheta2.reset()
	sliderTheta3.reset()
	sliderTheta4.reset()
	sliderTheta5.reset()
	sliderTheta6.reset()
	fig.canvas.draw_idle()

def resetCam(event):
	ax.view_init(azim=30)
	fig.canvas.draw_idle()

def updateFromSlider(val):
	thetas[0] = sliderTheta1.val
	thetas[1] = sliderTheta2.val
	thetas[2] = sliderTheta3.val
	thetas[3] = sliderTheta4.val
	thetas[4] = sliderTheta5.val
	thetas[5] = sliderTheta6.val
	# call updateMatrices and graph points, lines, axes, and labels at new joint positions
	# jointPositions, baseTransforms = ForwardK.ForwardK(thetas)[0::2]
	jointPositions, baseTransforms = ForwardK.updateMatrices(thetas)[0::2]
	drawArm(jointPositions, baseTransforms)

def drawArm(jointPositions, baseTransforms):
	# clears canvas and graphs new points, lines, axes, and labels
		# TODO only draw parts that have changed
	ax.clear()
	ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)  # Keeps z axis constant
	ax.axis([-400, 400, -400, 400])
	plotPointsLines(jointPositions)
	plotAxes(jointPositions, baseTransforms)
	drawLabels(jointPositions)
	ax.set_xlabel("x")
	ax.set_ylabel("y")
	ax.set_zlabel("z")
	fig.canvas.draw_idle()

# First test a group of random joint changes until one group makes the end effector closer to the goal position,
# then if no such joint angle change combinations are found quickly, check each joint change individually
# def IKTestTogether(text):
# 	global thetas
# 	np.set_printoptions(precision = 2)
# 	# draw specified point
# 	x, y, z = text.split(" ")
# 	print("x: {}, y: {}, z: {}".format(x, y, z))
# 	desiredToolPos = np.array([int(x), int(y), int(z)])

# 	# initial values and random theta directions
# 	maxRotation = 5  # specifies max degrees any joint is allowed to change by in a np.single frame
# 	rotation = maxRotation
# 	thresholdDistance = 50  # how precise we want to be
# 	initialThetas = thetas
# 	currentToolPos = ForwardK.getEndEffectorData(initialThetas)[0]  # initial position
# 	currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)  # initial distance
# 	initialDistance = currentDistance
# 	testDistance = 0

# 	while currentDistance > thresholdDistance:
# 		print("")
# 		print("current distance:")
# 		print(currentDistance)
# 		numTries = 0
# 		while numTries < 50:
# 			testChanges = np.random.rand(6)
# 			testThetas = thetas + testChanges * rotation
# 			currentJointPositions, baseTransforms = ForwardK.updateMatrices(testThetas)[0::2]  # 0 is joint positions
# 			currentToolPos = currentJointPositions[len(currentJointPositions)-1]  # tool position is last in the array of joint positions
# 			testDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)
# 			if testDistance < currentDistance:
# 				break
# 			else:
# 				numTries += 1
# 		if numTries == 50:
# 			for i in range(len(thetas)):
# 				testThetasCopy = thetas
# 				numnp.singleTries = 0
# 				while numnp.singleTries < 50:
# 					np.singleChange = np.random.rand(1)  						# 0.734
# 					np.singleTheta = thetas[i] + np.singleChange * rotation  	# 360 + 0.734 * rotation
# 					testThetasCopy[i] = np.singleTheta							# 382copy = 382
# 					# testThetasCopy = [360 382 360 360 360 360] only changing one at a time
# 					currentJointPositions, baseTransforms = ForwardK.updateMatrices(testThetasCopy)[0::2]
# 					currentToolPos = currentJointPositions[len(currentJointPositions)-1]
# 					testDistancenp.single = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)
# 					if testDistancenp.single < currentDistance:
# 						testThetas[i] = np.singleTheta  # if distance is less replace next theta for that joint
# 						break
# 					else:
# 						numnp.singleTries += 1
# 				if numnp.singleTries == 50:
# 					testThetas[i] = thetas[i]  # if distance is never less don't move theta for that joint

# 		thetas = testThetas

# 		# set new current distance
# 		currentJointPositions, baseTransforms = ForwardK.updateMatrices(thetas)[0::2]
# 		currentToolPos = currentJointPositions[len(currentJointPositions)-1]
# 		currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)

# 		# update amount allowed to rotate
# 		rotation = (currentDistance / initialDistance) * maxRotation + 1
# 		print("")
# 		print("maxRotation:")
# 		print(rotation)

# 		drawArm(currentJointPositions, baseTransforms)
# 		ax.scatter(int(x), int(y), int(z), s=70)
# 		plt.pause(.005)

# Change by a set number of degrees in each direction and take whichever one, if either, gives smaller distance
# update entire arm after each joint moves instead of after all joints have changed
# This works pretty well but is jittery near the end
def IKTestAtLeastOne(text):
	global thetas
	np.set_printoptions(precision = 2)
	# draw specified point
	x, y, z = text.split(" ")
	print("x: {}, y: {}, z: {}".format(x, y, z))
	desiredToolPos = np.array([int(x), int(y), int(z)])

	# initial values and random theta directions
	maxRotation = 5  # specifies max degrees any joint is allowed to change by in a np.single frame
	thresholdDistance = 50  # how precise we want to be
	initialThetas = thetas
	currentToolPos = ForwardK.getEndEffectorData(initialThetas)[0]  # initial position
	currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)  # initial distance
	initialDistance = currentDistance

	while currentDistance > thresholdDistance:
		noChange = np.full(6, False)
		for i in range(0, 3):
			thetasCopyAdd = np.copy(thetas)  # these arrays of thetas reset for each joint to evaluate each angle individually
			thetasCopySub = np.copy(thetas)
			thetasCopyAdd[i] += 5
			thetasCopySub[i] -= 5

			testDistanceAdd = getToolPositionDistance(thetasCopyAdd, desiredToolPos)[1]
			if testDistanceAdd < currentDistance:
				thetas[i] = thetasCopyAdd[i]
			else:
				testDistanceSub = getToolPositionDistance(thetasCopySub, desiredToolPos)[1]
				if testDistanceSub < currentDistance:
					thetas[i] = thetasCopySub[i]
				else:
					noChange[i] = True

		# set new current distance and update canvas with new joint positions
		currentJointPositions, baseTransforms = ForwardK.updateMatrices(thetas)[0::2]
		currentToolPos = currentJointPositions[len(currentJointPositions)-1]
		currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)

		# update amount allowed to rotate
		rotation = ((currentDistance / initialDistance) * maxRotation)**2 + 1
		print("")
		print("current Distance:")
		print(currentDistance)

		drawArm(currentJointPositions, baseTransforms)
		ax.scatter(int(x), int(y), int(z), s=70)
		plt.pause(.005)


# Test each joint change and update weights based on how each change affects the goal position distance
def IKTestIndividual(text):	
	global thetas
	np.set_printoptions(precision = 2)
	# draw specified point
	x, y, z = text.split(" ")
	print("x: {}, y: {}, z: {}".format(x, y, z))
	desiredToolPos = np.array([int(x), int(y), int(z)])

	# initial values and random theta directions
	maxRotation = 100  # specifies max degrees any joint is allowed to change by in a np.single frame
	thresholdDistance = 50  # how precise we want to be
	initialThetas = thetas
	thetaWeights = np.random.randn(6)
	thetaWeightsNext = np.zeros(6)
	currentToolPos = ForwardK.getEndEffectorData(initialThetas)[0]  # initial position
	currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)  # initial distance

	while currentDistance > thresholdDistance:
		print("")
		print("current distance:")
		print(currentDistance)
		print("")
		print("new distances:")
		# create new changes in theta
		testThetaDirs = np.vstack([thetas]*len(thetas))
		np.fill_diagonal(testThetaDirs, np.diagonal(testThetaDirs) + thetaWeights)
		print(testThetaDirs)

		# test changes in theta
		for i in range(len(testThetaDirs)):
			# get new end effector position as a result of only one theta changing
			testToolPosition = ForwardK.getEndEffectorData(testThetaDirs[i])[0]
			# find new distance for each theta to see how to see how to set weights
			newTestDistance = np.sqrt((desiredToolPos[0] - testToolPosition[0])**2 + (desiredToolPos[1] - testToolPosition[1])**2 + (desiredToolPos[2] - testToolPosition[2])**2)
			print(newTestDistance)
			print("")
			thetaWeightsNext[i] = (1 - newTestDistance / currentDistance)  # flip weight sign if new distance is bigger
		thetaDeltas = thetaWeightsNext * maxRotation
		thetaWeights = thetaDeltas + thetaWeights
		# print("theta weights: ")
		# # print(thetaWeights)
		# thetaDeltas = thetaWeights * maxRotation
		# print("theta deltas: ")
		# print(thetaDeltas)
		# update joint angles and find new cumulative end effector and joint positions
		thetas = thetaDeltas + thetas

		currentJointPositions, baseTransforms = ForwardK.updateMatrices(thetas)[0::2]  # 0 is joint positions
		# current tool position is all we currently care about for finding final joint angles
			# TODO figure out how to account for all other joint movements
		currentToolPos = currentJointPositions[len(currentJointPositions)-1]  # tool position is last in the array of joint positions
		currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)
		drawArm(currentJointPositions, baseTransforms)
		ax.scatter(int(x), int(y), int(z), s=70)
		plt.pause(.005)

		# get initial position
		# set change value for each theta
		# apply new temporary thetas individually to see if they bring end effector closer or not
		# set weight for each theta based on how close the new distances are
		# reverse weights that move end effector away
		# multiply weights by max joint rotation
		# add theta deltas to previous theta
		# update end effector position with new thetas
		# test if end effector is close enough to desired position
		# stop if true

	

# Inverse kinematics solution for 6-dof spherical wrist robot by skyentific
def SkyentificInverseK(input):
	global thetas
	np.set_printoptions(precision = 2)
	# draw desired end effector location
	x, y, z, a, b, g = input.split(" ")
	print("x: {}, y: {}, z: {}".format(x, y, z))
	# desiredToolPos = np.array([int(x), int(y), int(z)])
	
	Xik = np.array([float(x), float(y), float(z), float(a)*np.pi/180, float(b)*np.pi/180, float(g)*np.pi/180])
	Jik = np.zeros(6)	
 
	# inverse kinematics
	# input: Xik - pos value for the calculation of the inverse kinematics
	# output: Jik - joints value for the calculation of the inversed kinematics

	# Denavit-Hartenberg matrix
	theta=np.array([0.0, -90.0, 0.0, 0.0, 0.0, 0.0]) # theta=[0 -90+0 0 0 0 0]
	alfa=np.array([-90.0, 0.0, -90.0, 90.0, -90.0, 0.0]) # alfa=[-90 0 -90 90 -90 0]
	r=np.array([r1, r2, r3, 0.0, 0.0, 0.0]) # r=[47 110 26 0 0 0]
	d=np.array([d1, 0.0, d3, d4, 0.0, d6]) # d=[133 0 7 117.5 0 28]
 
	# from deg to rad
	mh.MatrixScale(theta, np.pi/180.0) # theta=theta*pi/180
	mh.MatrixScale(alfa, np.pi/180.0) # alfa=alfa*pi/180

	# work frame
	Xwf = np.zeros(6)

	# tool frame
	Xtf = np.zeros(6)

	# work frame transformation matrix
	Twf = mh.pos2tran(Xwf)
	print("Twf: {}".format(Twf))

	# tool frame transformation matrix
	Ttf = mh.pos2tran(Xtf)
	print("Ttf: {}".format(Ttf))

	# total transformation matrix
	Twt = mh.pos2tran(Xik)
	print("Twt: {}".format(Twt))

	# find T06
	inTwf = mh.invtran(Twf) # inTwf=mh.invtran(Twf)
	inTtf = mh.invtran(Ttf) # inTtf=mh.invtran(Ttf)
	print("inTwf: {}".format(inTwf))
	print("inTft: {}".format(inTtf))
	
	# np.matmul(Twt, inTtf, Tw6) # Tw6=Twt*inTtf
	Tw6 = Twt @ inTtf
	print("Tw6: {}".format(Tw6))
 
	# np.matmul(inTwf, Tw6, T06) # T06=inTwf*Tw6
	T06 = inTwf @ Tw6
	print("T06: {}".format(T06))

	# positon of the spherical wrist
	Xsw = np.array([T06[0][3]-d[5]*T06[0][2], T06[1][3]-d[5]*T06[1][2], T06[2][3]-d[5]*T06[2][2]])
	print("Wrist position: {}".format(Xsw))

	# Calculate joint angles
 
	# first joint
	Jik[0]=np.arctan2(Xsw[1],Xsw[0])-np.arctan2(d[2],np.sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])) # Jik(1)=np.arctan2(Xsw(2),Xsw(1))-np.arctan2(d(3),np.sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2))
	# second joint
	Jik[1]=np.pi/2.0
	-np.arccos((r[1]*r[1]+(Xsw[2]-d[0])*(Xsw[2]-d[0])+(np.sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(np.sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])-(r[2]*r[2]+d[3]*d[3]))/(2.0*r[1]*np.sqrt((Xsw[2]-d[0])*(Xsw[2]-d[0])+(np.sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(np.sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))))
	-np.arctan((Xsw[2]-d[0])/(np.sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])) # Jik(2)=pi/2-np.arccos((r(2)^2+(Xsw(3)-d(1))^2+(np.sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2-(r(3)^2+d(4)^2))/(2*r(2)*np.sqrt((Xsw(3)-d(1))^2+(np.sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)))-np.arctan((Xsw(3)-d(1))/(np.sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1)))
	# third joint
	Jik[2]=np.pi
	-np.arccos((r[1]*r[1]+r[2]*r[2]+d[3]*d[3]-(Xsw[2]-d[0])*(Xsw[2]-d[0])-(np.sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0])*(np.sqrt(Xsw[0]*Xsw[0]+Xsw[1]*Xsw[1]-d[2]*d[2])-r[0]))/(2*r[1]*np.sqrt(r[2]*r[2]+d[3]*d[3])))
	-np.arctan(d[3]/r[2]) # Jik(3)=pi-np.arccos((r(2)^2+r(3)^2+d(4)^2-(Xsw(3)-d(1))^2-(np.sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)/(2*r(2)*np.sqrt(r(3)^2+d(4)^2)))-np.arctan(d(4)/r(3))
	# last three joints
	T01 = np.zeros((4,4))
	T12 = np.zeros((4,4))
	T23 = np.zeros((4,4))
	T02 = np.zeros((4,4))
	T03 = np.zeros((4,4))
	inT03 = np.zeros((4,4))
	T36 = np.zeros((4,4))
	T01 = mh.DH1line(theta[0]+Jik[0], alfa[0], r[0], d[0]) # T01=DH1line(theta(1)+Jik(1),alfa(1),r(1),d(1))
	T12 = mh.DH1line(theta[1]+Jik[1], alfa[1], r[1], d[1]) # T12=DH1line(theta(2)+Jik(2),alfa(2),r(2),d(2))
	T23 = mh.DH1line(theta[2]+Jik[2], alfa[2], r[2], d[2]) # T23=DH1line(theta(3)+Jik(3),alfa(3),r(3),d(3))
	# np.matmul(T01, T12, T02)
	# np.matmul(T02, T23, T03)
	# np.matmul(inT03, T06, T36)
	
	T02 = T01 @ T12
	T03 = T02 @ T23
	T36 = inT03 @ T06
 
	inT03 = mh.invtran(T03) # inT03=mh.invtran(T03)
	# forth joint
	Jik[3]=np.arctan2(-T36[1][2], -T36[0][2]) # Jik(4)=np.arctan2(-T36(2,3),-T36(1,3))
	# fifth joint
	Jik[4]=np.arctan2(np.sqrt(T36[0][2]*T36[0][2]+T36[1][2]*T36[1][2]), T36[2][2]) # Jik(5)=np.arctan2(np.sqrt(T36(1,3)^2+T36(2,3)^2),T36(3,3))
	# sixth joints
	Jik[5]=np.arctan2(-T36[2][1], T36[2][0]) # Jik(6)=np.arctan2(-T36(3,2),T36(3,1))
	# rad to deg
	mh.MatrixScale(Jik, 180.0/np.pi) # Jik=Jik/pi*180
	thetas = Jik
 
	print("THETAS: {}".format(thetas))

	currentJointPositions, baseTransforms = ForwardK.updateMatrices(thetas)[0::2]
	print("JOINT POSITIONS: {}".format(currentJointPositions))
	drawArm(currentJointPositions, baseTransforms)
	
def InverseK(input):
	global thetas
	np.set_printoptions(precision = 2)
	# draw desired end effector location
	xEnd, yEnd, zEnd, alpha, beta, gamma = input.split(" ")
 
	xEnd = float(xEnd)
	yEnd = float(yEnd)
	zEnd = float(zEnd)
	alpha = float(alpha)
	beta = float(beta)
	gamma = float(gamma)
 
	lengths = np.array([135.7, 300.32, 293])
 
	transform0End = np.array([[math.cos(beta)*math.cos(gamma), -math.cos(beta)*math.sin(gamma), math.sin(beta), xEnd],
							  [math.cos(alpha)*math.sin(gamma) + math.cos(gamma)*math.sin(alpha)*math.sin(beta), math.cos(alpha)*math.cos(gamma) - math.sin(alpha)*math.sin(beta)*math.sin(gamma), -math.cos(beta)*math.sin(alpha), yEnd],
							  [-math.cos(alpha)*math.cos(gamma)*math.sin(beta) + math.sin(alpha)*math.sin(gamma), math.cos(alpha)*math.sin(beta)*math.sin(gamma) + math.cos(gamma)*math.sin(alpha), math.cos(alpha)*math.cos(beta), zEnd],
							  [0, 0, 0, 1]])
	transformEndSw = np.array([[1, 0, 0, 0],
							   [0, 1, 0, 0],
							   [0, 0, 1, d6-60],
							   [0, 0, 0, 1]])
	transform0Sw = transform0End @ transformEndSw

	sphericalPos = np.vstack(transform0Sw[:-1,3])
 
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
		theta2 = np.arccos((lengths[1]**2 - lengths[2]**2 + rSw**2)/(2*lengths[1]*rSw)) + alpha2
		theta3 = np.arccos((lengths[2]**2 + lengths[1]**2 - rSw**2)/(2*lengths[1]*lengths[2]))
		# Final three joint angles specify rotation only
		# Multi stuff:
		# dirEnd = Vector(xEnd, yEnd, zEnd, xSw, ySw, zSw, lenEnd)
		rElbow = lengths[1] * math.cos(theta2)
		xElbow = rElbow * math.cos(theta1)
		yElbow = rElbow * math.sin(theta1)
		zElbow = lengths[0] + lengths[1] * math.sin(theta2)
		dirForearm = Vector(xEnd, yEnd, zEnd, xElbow, yElbow, zElbow, lengths[2])
		print(dirForearm.get_direction())
		
		# remove list att from th 1 2 and 3
		theta1 = float(theta1)
		theta2 = float(theta2)
		theta3 = float(theta3)
  
		# rot_mat_0_6 = np.array([[-1.0, 0.0, 0.0],
		# 						[0.0, -1.0, 0.0],
		# 						[0.0, 0.0, 1.0]])
  
  		# rot_mat_0_3 = np.array([[-np.sin(theta1), 0.0, np.cos(theta1)],
		# 						[np.cos(theta1), 0.0, np.sin(theta1)],
		# 						[0.0, 1.0, 0.0]])

		rot_mat_0_6 = np.array([[np.cos(beta)*np.cos(gamma), -np.cos(beta)*np.sin(gamma), np.sin(beta)],
								[np.cos(alpha)*np.sin(gamma) + np.cos(gamma)*np.sin(alpha)*np.sin(beta), np.cos(alpha)*np.cos(gamma) - np.sin(alpha)*np.sin(beta)*np.sin(gamma), -np.cos(beta)*np.sin(alpha)],
								[-np.cos(alpha)*np.cos(gamma)*np.sin(beta) + np.sin(alpha)*np.sin(gamma), np.cos(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(gamma)*np.sin(alpha), np.cos(alpha)*np.cos(beta)]])


		rot_mat_0_3 = np.array([[np.cos(theta1)*np.cos(theta3)*(np.cos(theta2) - np.pi/2) + np.cos(theta1)*np.sin(theta3)*(-np.sin(theta2) - np.pi/2), np.cos(theta1)*np.cos(theta3)*(-np.sin(theta2) - np.pi/2) - np.cos(theta1)*np.sin(theta3)*(np.cos(theta2) - np.pi/2), -np.sin(theta1)],
								[np.cos(theta3)*np.sin(theta1)*(np.cos(theta2) - np.pi/2) + np.sin(theta1)*np.sin(theta3)*(-np.sin(theta2) - np.pi/2), np.cos(theta3)*np.sin(theta1)*(-np.sin(theta2) - np.pi/2) - np.sin(theta1)*np.sin(theta3)*(np.cos(theta2) - np.pi/2), np.cos(theta1)],
								[np.cos(theta3)*(-np.sin(theta2) - np.pi/2) + np.sin(theta3)*(-np.cos(theta2) - np.pi/2), np.cos(theta3)*(-np.cos(theta2) - np.pi/2) - np.sin(theta3)*(-np.sin(theta2) - np.pi/2), 0]])

		inv_rot_mat_0_3 = np.linalg.inv(rot_mat_0_3)
		rot_mat_3_6 = inv_rot_mat_0_3 @ rot_mat_0_6
  
		# theta5 = np.arccos(rot_mat_3_6[2, 2])
		# theta6 = np.arccos(rot_mat_3_6[2, 0] / -np.sin(theta5))
		# theta4 = np.arccos(rot_mat_3_6[1,2] / np.sin(theta5))
  
		theta5 = np.arccos(rot_mat_3_6[1][1])
		theta4 = -np.arccos(rot_mat_3_6[0][1]/np.sin(theta5))
		theta6 = np.arccos(rot_mat_3_6[1][0]/np.sin(theta5))
  
		# print(f'\nxSw: {xSw}, ySw: {ySw}, zSw: {zSw}')
		print(f'\nAngles in radians:\ntheta1: {theta1}\ntheta2: {theta2}\ntheta3: {theta3}\ntheta4: {theta4}\ntheta5: {theta5}\ntheta6: {theta6}')
		print(f'\nAngles in degrees:\ntheta1: {theta1*180/np.pi}\ntheta2: {theta2*180/np.pi}\ntheta3: {theta3*180/np.pi}\ntheta4: {theta4*180/np.pi}\ntheta5: {theta5*180/np.pi}\ntheta6: {theta6*180/np.pi}')
  
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
		currentJointPositions, baseTransforms = ForwardK.updateMatrices(thetas)[0::2]
		# print("JOINT POSITIONS: {}".format(currentJointPositions))
		drawArm(currentJointPositions, baseTransforms)

def getToolPositionDistance(thetas, desiredToolPos):
	currentToolPos = ForwardK.getEndEffectorData(thetas)[0]
	currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)
	return currentToolPos, currentDistance

def runToPosition(text):
	# IKTestIndividual(text)
	# IKTestTogether(text)
	# IKTestAtLeastOne(text)
	InverseK(text)
 
# get initial joint positions
# jointPositions, jointRotationMatrices, baseTransforms = ForwardK.updateMatrices(thetas)
jointPositions, jointRotationMatrices, baseTransforms = ForwardK.updateMatrices(thetas)

# Graph joint positions
fig = plt.figure(figsize=(5, 6))
ax = fig.add_subplot(projection='3d')
plt.subplots_adjust(bottom=0.4)
maxZ = 400
minZ = -400
plotPointsLines(jointPositions)
drawLabels(jointPositions)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.view_init(azim=30)
plotAxes(jointPositions, baseTransforms)
# Add slider for each axis
axtheta1 = plt.axes([0.15, 0.35, 0.65, 0.03])
axtheta2 = plt.axes([0.15, 0.3, 0.65, 0.03])
axtheta3 = plt.axes([0.15, 0.25, 0.65, 0.03])
axtheta4 = plt.axes([0.15, 0.2, 0.65, 0.03])
axtheta5 = plt.axes([0.15, 0.15, 0.65, 0.03])
axtheta6 = plt.axes([0.15, 0.1, 0.65, 0.03])
axReset = plt.axes([0.05, 0.8, 0.08, 0.05])
camReset = plt.axes([0.05, 0.88, 0.15, 0.05])
xyzInput = plt.axes([0.2, 0.04, 0.55, 0.04])
ax.axis([-400, 400, -400, 400])
ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)  # Z points to keep axis consistent
# Slider and button creation
sliderTheta1 = Slider(axtheta1, 'Theta1', 0, 360*numRotations, valinit=360*numRotations/2)
sliderTheta2 = Slider(axtheta2, 'Theta2', 0, 360*numRotations, valinit=360*numRotations/2)
sliderTheta3 = Slider(axtheta3, 'Theta3', 0, 360*numRotations, valinit=360*numRotations/2)
sliderTheta4 = Slider(axtheta4, 'Theta4', 0, 360*numRotations, valinit=360*numRotations/2)
sliderTheta5 = Slider(axtheta5, 'Theta5', 0, 360*numRotations, valinit=360*numRotations/2)
sliderTheta6 = Slider(axtheta6, 'Theta6', 0, 360*numRotations, valinit=360*numRotations/2)
resetButton = Button(axReset, 'Reset')
resetCameraButton = Button(camReset, 'Reset Cam')
xyzInputText = TextBox(xyzInput, 'Desired pos')

# Slider and button updates
sliderTheta1.on_changed(updateFromSlider)
sliderTheta2.on_changed(updateFromSlider)
sliderTheta3.on_changed(updateFromSlider)
sliderTheta4.on_changed(updateFromSlider)
sliderTheta5.on_changed(updateFromSlider)
sliderTheta6.on_changed(updateFromSlider)
resetButton.on_clicked(reset)
resetCameraButton.on_clicked(resetCam)
xyzInputText.on_submit(runToPosition)

plt.show()

if __name__ == '__main__':
	pass


