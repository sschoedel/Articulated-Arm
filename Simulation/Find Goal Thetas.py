import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.widgets import Slider, Button, TextBox
import ForwardKinematics as ForwardK

plt.style.use('seaborn-whitegrid')

numRotations = 2

# Initial theta inputs in degrees
thetas = np.array([360*numRotations/2] * 6)

# get initial joint positions
jointPositions, jointRotationMatrices, baseTransforms = ForwardK.updateMatrices(thetas)

# functions for widgets
# reset joint rotations button
def reset(event):
	sliderTheta1.reset()
	sliderTheta2.reset()
	sliderTheta3.reset()
	sliderTheta4.reset()
	sliderTheta5.reset()
	sliderTheta6.reset()
	fig.canvas.draw_idle()

# reset camera angle button
def resetCam(event):
	ax.view_init(azim=30)
	fig.canvas.draw_idle()

# toggle individual joint axes
def toggleAxes(event):
	global drawAxesBool
	if drawAxesBool:
		drawAxesBool = False
	else:
		drawAxesBool = True
	jointPositions, baseTransforms = ForwardK.updateMatrices(thetas)[0::2]
	drawArm(jointPositions, baseTransforms)

def toggleTrailLines(event):
	global trailLinesBool
	if trailLinesBool:
		trailLinesBool = False
	else:
		trailLinesBool = True
	jointPositions, baseTransforms = ForwardK.updateMatrices(thetas)[0::2]
	drawArm(jointPositions, baseTransforms)

# update thetas from sliders
def updateFromSlider(val):
	thetas[0] = sliderTheta1.val
	thetas[1] = sliderTheta2.val
	thetas[2] = sliderTheta3.val
	thetas[3] = sliderTheta4.val
	thetas[4] = sliderTheta5.val
	thetas[5] = sliderTheta6.val
	# call updateMatrices and graph points, lines, axes, and labels at new joint positions
	jointPositions, baseTransforms = ForwardK.updateMatrices(thetas)[0::2]
	drawArm(jointPositions, baseTransforms)

# Test each joint change and update weights based on how each change effects the goal position distance
def IKTestIndividual(text):	
	global thetas
	np.set_printoptions(precision = 2)
	# draw specified point
	x, y, z = text.split(" ")
	print("x: {}, y: {}, z: {}".format(x, y, z))
	desiredToolPos = np.array([int(x), int(y), int(z)])

	# initial values and random theta directions
	maxRotation = 100  # specifies max degrees any joint is allowed to change by in a single frame
	thresholdDistance = 50  # how precise we want to be
	initialThetas = thetas
	thetaWeights = np.random.randn(6)
	thetaWeightsNext = np.zeros(6)
	currentToolPos = ForwardK.getEndEffectorData(initialThetas)[0]  # initial position
	currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)  # initial distance

	while currentDistance > thresholdDistance:
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

def getToolPositionDistance(thetas, desiredToolPos):
		currentToolPos = ForwardK.getEndEffectorData(thetas)[0]
		currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)
		return currentToolPos, currentDistance

def runToPosition():
	IKTestAtLeastOne(text)

plt.show()

if __name__ == '__main__':
	pass