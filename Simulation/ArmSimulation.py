import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.widgets import Slider, Button, TextBox
import ForwardKinematics as ForwardK

plt.style.use('seaborn-whitegrid')

numRotations = 2

# Initial theta inputs in degrees
thetas = np.array([360*numRotations/2] * 6)

def getRotations(jointRotationMatrices):
	# equations for y x z rotations from rotation matrix
	# beta = np.atan2(-r31, +-np.sqrt(r11** + r21**))
	# alpha = np.atan2(r21/cos(beta), r11/cos(beta))
	# gamma = np.atan2(r32/cos(beta), r33/cos(beta))

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

# get initial joint positions
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

def IKTestTogether(text):
	global thetas
	np.set_printoptions(precision = 2)
	# draw specified point
	x, y, z = text.split(" ")
	print("x: {}, y: {}, z: {}".format(x, y, z))
	desiredToolPos = np.array([int(x), int(y), int(z)])

	# initial values and random theta directions
	maxRotation = 5  # specifies max degrees any joint is allowed to change by in a single frame
	rotation = maxRotation
	thresholdDistance = 50  # how precise we want to be
	initialThetas = thetas
	currentToolPos = ForwardK.getEndEffectorData(initialThetas)[0]  # initial position
	currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)  # initial distance
	initialDistance = currentDistance
	testDistance = 0

	while currentDistance > thresholdDistance:
		print("")
		print("current distance:")
		print(currentDistance)
		numTries = 0
		while numTries < 50:
			testChanges = np.random.rand(6)
			testThetas = thetas + testChanges * rotation
			currentJointPositions, baseTransforms = ForwardK.updateMatrices(testThetas)[0::2]  # 0 is joint positions
			currentToolPos = currentJointPositions[len(currentJointPositions)-1]  # tool position is last in the array of joint positions
			testDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)
			if testDistance < currentDistance:
				break
			else:
				numTries = numTries + 1
		thetas = testThetas
		currentDistance = testDistance
		maxRotation = (currentDistance / initialDistance) * rotation + 1
		print("")
		print("maxRotation:")
		print(maxRotation)

		drawArm(currentJointPositions, baseTransforms)
		ax.scatter(int(x), int(y), int(z), s=70)
		plt.pause(.005)

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


def runToPosition(text):
	# IKTestIndividual(text)
	IKTestTogether(text)


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