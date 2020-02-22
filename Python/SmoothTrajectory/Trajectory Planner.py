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

xyzInput = plt.axes([0.2, 0.04, 0.55, 0.04])
changeN = plt.axes([0.2, 0.1, 0.2, 0.04])
axDrawButton = plt.axes([0.2, 0.16, 0.2, 0.04])
xyzInputText = TextBox(xyzInput, 'Desired Pos: ')
changeNText = TextBox(changeN, 'N: ')
drawTrajectory = Button(axDrawButton, 'Start')

def getToolPositionDistance(thetas, desiredToolPos):
		currentToolPos = ForwardK.getEndEffectorData(thetas)[0]
		currentDistance = np.sqrt((desiredToolPos[0] - currentToolPos[0])**2 + (desiredToolPos[1] - currentToolPos[1])**2 + (desiredToolPos[2] - currentToolPos[2])**2)
		return currentToolPos, currentDistance

def findEndThetas():
	global thetas
	# initial values and random theta directions
	maxRotation = 2  # specifies max degrees any joint is allowed to change by in a single frame
	rotation = maxRotation
	thresholdDistance = 50  # how precise we want to be
	initialThetas = thetas
	currentToolPos = ForwardK.getEndEffectorData(initialThetas)[0]  # initial position
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

	thetaend = thetas
	return thetaend

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
	if diffx is not 0 and diffy is not 0 and diffz is not 0:
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
		print("pos {}".format(i+1))
		plt.pause(0.005)
	# reset everything TODO fix errors this might be causing
	thetastart = thetaend
	thetaend = np.zeros(6)
	trail = np.empty([1,3])

def changeN(text):
	global N
	N = int(text)

def changeGoalPosition(text):
	global desiredToolPos
	np.set_printoptions(precision = 2)
	x, y, z = text.split(" ")
	print("x: {}, y: {}, z: {}".format(x, y, z))
	desiredToolPos = np.array([int(x), int(y), int(z)])


xyzInputText.on_submit(changeGoalPosition)
drawTrajectory.on_clicked(start)
changeNText.on_submit(changeN)

plt.show()