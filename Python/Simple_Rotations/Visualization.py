import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox

plt.style.use('seaborn-whitegrid')

disp = [0, 0, 0]

# functions for widgets
def reset(event):
	sliderAlpha.reset()
	sliderBeta.reset()
	sliderGamma.reset()
	sliderXend.reset()
	sliderYend.reset()
	sliderZend.reset()
	fig.canvas.draw_idle()

def resetCam(event):
	ax.view_init(azim=30)
	fig.canvas.draw_idle()

def updateFromSlider(val):
	A = sliderAlpha.val * np.pi/180
	B = sliderBeta.val * np.pi/180
	G = sliderGamma.val * np.pi/180
	disp[0] = sliderXend.val
	disp[1] = sliderYend.val
	disp[2] = sliderZend.val
 
	rotate_x = np.array(   [[1, 0, 0],
							[0, np.cos(A), -np.sin(A)],
							[0, np.sin(A), np.cos(A)]])
	rotate_y = np.array(   [[np.cos(B), 0, np.sin(B)],
							[0, 1, 0],
							[-np.sin(B), 0, np.cos(B)]])
	rotate_z = np.array(   [[np.cos(G), -np.sin(G), 0], 
							[np.sin(G), np.cos(G), 0],
							[0, 0, 1]])

	# rotate_mat = rotate_x @ rotate_y @ rotate_z
	rotate_mat = rotate_y @ rotate_x

	transform = np.append(rotate_mat, [[0,0,0]], 0)
	transform = np.append(transform, [[disp[0]], [disp[1]], [disp[2]], [1]], axis=1)
 
	print(transform)
 
	# Extract position and rotation data
	position = np.array([transform[:-1,3]]).flatten()
	rotationMatrix = np.array([transform[:-1,:-1]])
 
	print(position)
	print(rotationMatrix)


	# plot stuff
	ax.clear()
	ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)  # Keeps z axis constant
	ax.axis([-400, 400, -400, 400])
 
	ax.scatter(position[0], position[1], position[2], color='.6')
	ax.scatter(0, 0, 0, color='.3')
 
	xline = [position[0], 0]
	yline = [position[1], 0]
	zline = [position[2], 0]
	ax.plot3D(xline, yline, zline, 'chartreuse')
 
	axesXYZsGlobal = np.array([[300, 0, 0],[0, 300, 0],[0, 0, 300]])  # global xyz coordinates
	axesXYZsGlobalMatrix = np.array([[[1, 0, 0, axesXYZsGlobal[i,0]],  # make matrix out of global xyz coords
						  	 		  [0, 1, 0, axesXYZsGlobal[i,1]],
						  			  [0, 0, 1, axesXYZsGlobal[i,2]],
						  	 		  [0, 0, 0, 1]] for i in range(0, axesXYZsGlobal.shape[0])])
	I = np.identity(4)
	locations = np.array([I, transform])
	axesXYZsLocalMatrix = [locations[t] @ axesXYZsGlobalMatrix[i]  for t in range(0, locations.shape[0]) for i in range(0, axesXYZsGlobal.shape[0])]  # transform into each joint's coords
	axesXYZsLocalMatrix = np.reshape(axesXYZsLocalMatrix, (6, 4, -1))  # reshape for usability
	axesXYZsLocal = np.array([axesXYZsLocalMatrix[i,:-1,3] for i in range(0, axesXYZsLocalMatrix.shape[0])])  # extract position data from matrix
 
	localXAxes = axesXYZsLocal[0::3]
	localYAxes = axesXYZsLocal[1::3]
	localZAxes = axesXYZsLocal[2::3]
 
	jointPositions = np.array([[0,0,0], position])
 
	[ax.plot3D((jointPositions[i,0], localXAxes[i,0]),(jointPositions[i,1], localXAxes[i,1]),(jointPositions[i,2], localXAxes[i,2]), 'red') for i in range(0,2)]
	[ax.plot3D((jointPositions[i,0], localYAxes[i,0]),(jointPositions[i,1], localYAxes[i,1]),(jointPositions[i,2], localYAxes[i,2]), 'green') for i in range(0,2)]
	[ax.plot3D((jointPositions[i,0], localZAxes[i,0]),(jointPositions[i,1], localZAxes[i,1]),(jointPositions[i,2], localZAxes[i,2]), 'blue') for i in range(0,2)]
 
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
xend = plt.axes([0.15, 0.2, 0.65, 0.03])
yend = plt.axes([0.15, 0.15, 0.65, 0.03])
zend = plt.axes([0.15, 0.1, 0.65, 0.03])

#buttons
axReset = plt.axes([0.05, 0.8, 0.08, 0.05])
camReset = plt.axes([0.05, 0.88, 0.15, 0.05])

ax.axis([-400, 400, -400, 400])
ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)  # Z points to keep axis consistent

sliderAlpha = Slider(alpha, 'alpha', -180, 180, valinit=0)
sliderBeta  = Slider(beta, 'beta', -180, 180, valinit=0)
sliderGamma = Slider(gamma, 'gamma', -180, 180, valinit=0)
sliderXend = Slider(xend, 'xend', -400, 400, valinit=disp[0])
sliderYend  = Slider(yend, 'yend', -400, 400, valinit=disp[1])
sliderZend = Slider(zend, 'zend', -400, 400, valinit=disp[2])
resetButton = Button(axReset, 'Reset')
resetCameraButton = Button(camReset, 'Reset Cam')

#updates
sliderAlpha.on_changed(updateFromSlider)
sliderBeta.on_changed(updateFromSlider)
sliderGamma.on_changed(updateFromSlider)
sliderXend.on_changed(updateFromSlider)
sliderYend.on_changed(updateFromSlider)
sliderZend.on_changed(updateFromSlider)
resetButton.on_clicked(reset)
resetCameraButton.on_clicked(resetCam)

plt.show()

if __name__ == '__main__':
	pass
