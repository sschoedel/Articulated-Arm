import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.widgets import Slider, Button, TextBox

plt.style.use('seaborn-whitegrid')

endEffector = [0, 0, 60]
endPosition = [400, 350, 500]
base = [0, 0, 0]

# functions for widgets
def reset(event):
	sliderAlpha.reset()
	sliderBeta.reset()
	sliderGamma.reset()
	fig.canvas.draw_idle()

def resetCam(event):
	ax.view_init(azim=30)
	fig.canvas.draw_idle()

def updateFromSlider(val):
	A = sliderAlpha.val * np.pi/180
	B = sliderBeta.val * np.pi/180
	G = sliderGamma.val * np.pi/180
 
 
    # calculate position of wrist

	transform0End = np.array([[np.cos(B)*np.cos(G), -np.cos(B)*np.sin(G), np.sin(B), endPosition[0]],
							[np.cos(A)*np.sin(G) + np.cos(G)*np.sin(A)*np.sin(B), np.cos(A)*np.cos(G) - np.sin(A)*np.sin(B)*np.sin(G), -np.cos(B)*np.sin(A), endPosition[1]],
							[-np.cos(A)*np.cos(G)*np.sin(B) + np.sin(A)*np.sin(G), np.cos(A)*np.sin(B)*np.sin(G) + np.cos(G)*np.sin(A), np.cos(A)*np.cos(B), endPosition[2]],
							[0, 0, 0, 1]])
	transformEndSw = np.array([[1, 0, 0, -endEffector[0]],
							[0, 1, 0, -endEffector[1]],
							[0, 0, 1, -endEffector[2]],
							[0, 0, 0, 1]])
	transform0Sw = transform0End @ transformEndSw

	sphericalPos = np.hstack(transform0Sw[:-1,3])
 
	ax.clear()
	ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)  # Keeps z axis constant
	ax.axis([-400, 400, -400, 400])
	
	ax.scatter(sphericalPos[0], sphericalPos[1], sphericalPos[2], color='.3')
	ax.scatter(0, 1, 2, color='.3')
	ax.scatter(endPosition[0], endPosition[1], endPosition[2], color='.6')
	xline = [sphericalPos[0], endPosition[0]]
	yline = [sphericalPos[1], endPosition[1]]
	zline = [sphericalPos[2], endPosition[2]]	
	ax.plot3D(xline, yline, zline, 'chartreuse')
 
 
	axesXYZsGlobal = np.array([[50, 0, 0],[0, 50, 0],[0, 0, 50]])  # global xyz coordinates
	axesXYZsGlobalMatrix = np.array([[[1, 0, 0, axesXYZsGlobal[i,0]],  # make matrix out of global xyz coords
						  	 		  [0, 1, 0, axesXYZsGlobal[i,1]],
						  			  [0, 0, 1, axesXYZsGlobal[i,2]],
						  	 		  [0, 0, 0, 1]] for i in range(0, axesXYZsGlobal.shape[0])])
	locations = np.array([transform0Sw, transform0End])
	axesXYZsLocalMatrix = [locations[t] @ axesXYZsGlobalMatrix[i]  for t in range(0, locations.shape[0]) for i in range(0, axesXYZsGlobal.shape[0])]  # transform into each joint's coords
	axesXYZsLocalMatrix = np.reshape(axesXYZsLocalMatrix, (6, 4, -1))  # reshape for usability
	axesXYZsLocal = np.array([axesXYZsLocalMatrix[i,:-1,3] for i in range(0, axesXYZsLocalMatrix.shape[0])])  # extract position data from matrix
 
	localXAxes = axesXYZsLocal[0::3]
	localYAxes = axesXYZsLocal[1::3]
	localZAxes = axesXYZsLocal[2::3]
 
	jointPositions = np.array([sphericalPos, endPosition])
 
	[ax.plot3D((jointPositions[i,0], localXAxes[i,0]),(jointPositions[i,1], localXAxes[i,1]),(jointPositions[i,2], localXAxes[i,2]), 'red') for i in range(0,1)]
	[ax.plot3D((jointPositions[i,0], localYAxes[i,0]),(jointPositions[i,1], localYAxes[i,1]),(jointPositions[i,2], localYAxes[i,2]), 'green') for i in range(0,1)]
	[ax.plot3D((jointPositions[i,0], localZAxes[i,0]),(jointPositions[i,1], localZAxes[i,1]),(jointPositions[i,2], localZAxes[i,2]), 'blue') for i in range(0,1)]
 
	ax.set_xlabel("x")
	ax.set_ylabel("y")
	ax.set_zlabel("z")
	fig.canvas.draw_idle()
 
 
fig = plt.figure(figsize=(5, 6))
ax = fig.add_subplot(projection='3d')
plt.subplots_adjust(bottom=0.4)
maxZ = 400
minZ = -400
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.view_init(azim=30)

#sliders
alpha = plt.axes([0.15, 0.35, 0.65, 0.03])
beta = plt.axes([0.15, 0.3, 0.65, 0.03])
gamma = plt.axes([0.15, 0.25, 0.65, 0.03])

#buttons
axReset = plt.axes([0.05, 0.8, 0.08, 0.05])
camReset = plt.axes([0.05, 0.88, 0.15, 0.05])

ax.axis([-400, 400, -400, 400])
ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)  # Z points to keep axis consistent

sliderAlpha = Slider(alpha, 'alpha', 0, 360, valinit=360)
sliderBeta  = Slider(beta, 'beta', 0, 360, valinit=360)
sliderGamma = Slider(gamma, 'gamma', 0, 360, valinit=360)
resetButton = Button(axReset, 'Reset')
resetCameraButton = Button(camReset, 'Reset Cam')

#updates
sliderAlpha.on_changed(updateFromSlider)
sliderBeta.on_changed(updateFromSlider)
sliderGamma.on_changed(updateFromSlider)
resetButton.on_clicked(reset)
resetCameraButton.on_clicked(resetCam)

plt.show()

if __name__ == '__main__':
	pass
