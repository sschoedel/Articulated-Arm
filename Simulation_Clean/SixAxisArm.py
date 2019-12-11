import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.widgets import Slider, Button, TextBox

plt.style.use('seaborn-whitegrid')

class Arm:
	"""Robotic arm simulation"""

	def __init__(self, numSliderRotations):
		self.thetas = np.array([360*numSliderRotations/2] * 6)  # joint angles
		self.trail = np.empty([1,3])				# holds trail data
		self.prevEndPosition = np.array([0, 0, 0])  # used in drawing trail
		self.drawAxesBool = True		# used to toggle display information
		self.trailLinesBool = False		# used to toggle display information
		self.jointPositions, self.jointRotationMatrices, self.baseTransforms = ForwardK.updateMatrices(self.thetas)
		self.maxZ = 400
		self.minZ = -400
		self.fig = plt.figure(figsize=(5, 6))
		self.ax = fig.add_subplot(projection='3d')

	def getEndEffectorData():
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

	def updateMatrices():
		theta = theta * np.pi/180
		
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

	def printMatrices(jointPositions, jointAngles):
		print("joint positions")
		print(jointPositions)
		print("")
		print("joint angles")
		print(jointAngles)

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

	def getToolPositionDistance(desiredToolPos):
			currentToolPos = ForwardK.getEndEffectorData(self.thetas)[0]
			currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)
			return currentToolPos, currentDistance

	def drawLabels(jointPositions):
		jointPositions = np.insert(jointPositions, 0, [0, 0, 0], axis=0)
		ax.text(jointPositions[0,0], jointPositions[0,1], jointPositions[0,2], "origin", color='orange')
		for i in range(1, 7):
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

		# draw x, y, and z lines for each axis
		[ax.plot3D((jointPositions[i,0], localXAxes[i,0]),(jointPositions[i,1], localXAxes[i,1]),(jointPositions[i,2], localXAxes[i,2]), 'red') for i in range(0,7)]
		[ax.plot3D((jointPositions[i,0], localYAxes[i,0]),(jointPositions[i,1], localYAxes[i,1]),(jointPositions[i,2], localYAxes[i,2]), 'green') for i in range(0,7)]
		[ax.plot3D((jointPositions[i,0], localZAxes[i,0]),(jointPositions[i,1], localZAxes[i,1]),(jointPositions[i,2], localZAxes[i,2]), 'blue') for i in range(0,7)]

	# responsible for drawing arm joints, arm segments, joint axes, joint labels, end effector trail, and trail labels
	def drawArm(jointPositions, baseTransforms):
		# clears canvas and graphs new points, lines, axes, and labels
			# TODO only draw parts that have changed
		ax.clear()
		ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)  # Keeps z axis constant
		ax.axis([-400, 400, -400, 400])
		plotPointsLines(jointPositions)
		if self.drawAxesBool:
			plotAxes(jointPositions, baseTransforms)
		drawLabels(jointPositions)
		drawTrail(jointPositions)
		drawTrailLines()
		ax.set_xlabel("x")
		ax.set_ylabel("y")
		ax.set_zlabel("z")
		fig.canvas.draw_idle()

	# draw and label points where the end effector has been
	def drawTrail(jointPositions):
		currentEndPosition = jointPositions[len(jointPositions)-1]
		diffx = int(abs(self.prevEndPosition[0] - currentEndPosition[0]))
		diffy = int(abs(self.prevEndPosition[1] - currentEndPosition[1]))
		diffz = int(abs(self.prevEndPosition[2] - currentEndPosition[2]))
		if diffx is not 0 and diffy is not 0 and diffz is not 0:
			self.trail = np.append(self.trail, [currentEndPosition], axis=0)
			self.prevEndPosition = currentEndPosition
		ax.scatter(self.trail[:,0], self.trail[:,1], self.trail[:,2], s=10, color="orange")
		for i in range(1, len(self.trail)):
			ax.text(self.trail[i,0], self.trail[i,1], self.trail[i,2], i, color='red')
		self.prevEndPosition = currentEndPosition

	def drawTrailLines():
		if self.trailLinesBool:
			ax.plot3D(self.self.trail[1:,0], self.self.trail[1:,1], self.self.trail[1:,2], color="orange")

	def solveIK(text):
		np.set_printoptions(precision = 2)
		# draw specified point
		x, y, z = text.split(" ")
		print("x: {}, y: {}, z: {}".format(x, y, z))
		desiredToolPos = np.array([int(x), int(y), int(z)])

		# initial values and random theta directions
		maxRotation = 2  # specifies max degrees any joint is allowed to change by in a single frame
		rotation = maxRotation
		thresholdDistance = 50  # how precise we want to be
		currentToolPos = ForwardK.getEndEffectorData(self.thetas)[0]  # initial position
		currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)  # initial distance
		initialDistance = currentDistance
		testDistance = 0

		while currentDistance > thresholdDistance:
			noChange = np.full(6, False)
			for i in range(len(thetas)):
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

			drawArm(currentJointPositions, baseTransforms)
			ax.scatter(int(x), int(y), int(z), s=70)
			plt.pause(.005)

	def showPlot():
		plt.show()


	# Graph joint positions
	plt.subplots_adjust(bottom=0.4)
	plotPointsLines(self.jointPositions)
	drawLabels(self.jointPositions)
	self.ax.set_xlabel("x")
	self.ax.set_ylabel("y")
	self.ax.set_zlabel("z")
	self.ax.view_init(azim=30)
	plotAxes(self.jointPositions, self.baseTransforms)
	# Add slider for each axis
	axtheta1 = plt.axes([0.15, 0.35, 0.65, 0.03])
	axtheta2 = plt.axes([0.15, 0.3, 0.65, 0.03])
	axtheta3 = plt.axes([0.15, 0.25, 0.65, 0.03])
	axtheta4 = plt.axes([0.15, 0.2, 0.65, 0.03])
	axtheta5 = plt.axes([0.15, 0.15, 0.65, 0.03])
	axtheta6 = plt.axes([0.15, 0.1, 0.65, 0.03])
	axReset = plt.axes([0.05, 0.9, 0.08, 0.05])
	axCamReset = plt.axes([0.16, 0.9, 0.15, 0.05])
	axShowAxes = plt.axes([0.34, 0.9, 0.17, 0.05])
	axShowTrailLines = plt.axes([0.54, 0.9, 0.23, 0.05])
	xyzInput = plt.axes([0.2, 0.04, 0.55, 0.04])
	self.ax.axis([-400, 400, -400, 400])
	self.ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)  # Z points to keep axis consistent
	# Slider and button creation
	sliderTheta1 = Slider(axtheta1, 'Theta1', 0, 360*numRotations, valinit=360*numRotations/2)
	sliderTheta2 = Slider(axtheta2, 'Theta2', 0, 360*numRotations, valinit=360*numRotations/2)
	sliderTheta3 = Slider(axtheta3, 'Theta3', 0, 360*numRotations, valinit=360*numRotations/2)
	sliderTheta4 = Slider(axtheta4, 'Theta4', 0, 360*numRotations, valinit=360*numRotations/2)
	sliderTheta5 = Slider(axtheta5, 'Theta5', 0, 360*numRotations, valinit=360*numRotations/2)
	sliderTheta6 = Slider(axtheta6, 'Theta6', 0, 360*numRotations, valinit=360*numRotations/2)
	resetButton = Button(axReset, 'Reset')
	resetCameraButton = Button(axCamReset, 'Reset Cam')
	showAxesButton = Button(axShowAxes, 'Toggle Axes')
	showTrailLinesButton = Button(axShowTrailLines, 'Toggle Trail Lines')
	xyzInputText = TextBox(xyzInput, 'Desired Pos')

	# Slider and button updates
	sliderTheta1.on_changed(updateFromSlider)
	sliderTheta2.on_changed(updateFromSlider)
	sliderTheta3.on_changed(updateFromSlider)
	sliderTheta4.on_changed(updateFromSlider)
	sliderTheta5.on_changed(updateFromSlider)
	sliderTheta6.on_changed(updateFromSlider)
	resetButton.on_clicked(reset)
	resetCameraButton.on_clicked(resetCam)
	showAxesButton.on_clicked(toggleAxes)
	showTrailLinesButton.on_clicked(toggleTrailLines)
	xyzInputText.on_submit(solveIK)

if __name__ == '__main__':
	pass