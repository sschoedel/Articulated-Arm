import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.widgets import Slider, Button, TextBox
plt.style.use('seaborn-whitegrid')

numRotations = 2

# Theta inputs in degrees
z1=360*numRotations/2
z2=360*numRotations/2
z3=360*numRotations/2
z4=360*numRotations/2
z5=360*numRotations/2
z6=360*numRotations/2

def updateMatrices(z1,z2, z3, z4, z5, z6):
	theta = np.array([z1 * np.pi/180, z2 * np.pi/180, z3 * np.pi/180, z4 * np.pi/180, z5 * np.pi/180, z6 * np.pi/180])
	alpha = np.array([0, -np.pi/2, 0, -np.pi/2, -np.pi/2, np.pi/2])

	toolPosition = np.array([0, 0, 60])

	xOff = np.array([22.12, 0, 300.32, 0, 0, 0])
	yOff = np.array([0, 31.8, 0, 293, 0, -62])
	zOff = np.array([135.7, 0, 0, -36.3, 0, 0])

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

	# beta = np.atan2(-r31, +-np.sqrt(r11** + r21**))
	# alpha = np.atan2(r21/cos(beta), r11/cos(beta))
	# yeta = np.atan2(r32/cos(beta), r33/cos(beta))

	np.set_printoptions(linewidth=100, precision=2, suppress=True)
	# Extract position and rotation data for each joint
	jointPositions = np.array([baseTransforms[i,:-1,3] for i in range(0, baseTransforms.shape[0])])
	jointRotationMatrices = np.array([baseTransforms[i,:-1,:-1] for i in range(0, baseTransforms.shape[0])])
	return jointPositions, baseTransforms, jointRotationMatrices

def getRotations(jointRotationMatrices):
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

jointPositions, baseTransforms, jointRotationMatrices = updateMatrices(z1, z2, z3, z4, z5, z6)

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

def resetCam(event):
	ax.view_init(azim=30)

def submit(text):
	print(text)

def update(val):
	theta1 = sliderTheta1.val
	theta2 = sliderTheta2.val
	theta3 = sliderTheta3.val
	theta4 = sliderTheta4.val
	theta5 = sliderTheta5.val
	theta6 = sliderTheta6.val
	# call updateMatrices and graph points and lines at those new positions
	jointPositions, baseTransforms, jointRotationMatrices = updateMatrices(theta1, theta2, theta3, theta4, theta5, theta6)
	ax.clear()
	ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)  # Keeps z axis constant
	ax.axis([-400, 400, -400, 400])
	plotPointsLines(jointPositions)
	jointAnglesRad, jointAnglesTheta = getRotations(jointRotationMatrices)
	plotAxes(jointPositions, baseTransforms)
	drawLabels(jointPositions)
	# printMatrices(jointPositions, jointAnglesTheta)
	ax.set_xlabel("x")
	ax.set_ylabel("y")
	ax.set_zlabel("z")
	fig.canvas.draw_idle()

# Slider and button updates
sliderTheta1.on_changed(update)
sliderTheta2.on_changed(update)
sliderTheta3.on_changed(update)
sliderTheta4.on_changed(update)
sliderTheta5.on_changed(update)
sliderTheta6.on_changed(update)
resetButton.on_clicked(reset)
resetCameraButton.on_clicked(resetCam)
xyzInputText.on_submit(submit)

plt.show()