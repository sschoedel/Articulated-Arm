import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.widgets import Slider, Button, TextBox

plt.style.use('seaborn-whitegrid')

endEffector = [60, 0, 0]
endPosition = [150, 0, 300]
base = [0, 0, 0]

r1_1 = 22.12
r2_1 = 135.7
r3_1 = 31.8
d1_1 = 300.32
d3_1 = -36.3
d4_1 = 293
d6_1 = 62

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
	print(f'A, B, G: {[A, B, G]}')
	endPosition[0] = sliderXend.val
	endPosition[1] = sliderYend.val
	endPosition[2] = sliderZend.val
	lengths = np.array([135.7, 300.32, 293])
 
 
    # calculate position of wrist

	# transform0End = np.array([[np.cos(B)*np.cos(G), -np.cos(B)*np.sin(G), np.sin(B), endPosition[0]],
	# 						  [np.cos(A)*np.sin(G) + np.cos(G)*np.sin(A)*np.sin(B), np.cos(A)*np.cos(G) - np.sin(A)*np.sin(B)*np.sin(G), -np.cos(B)*np.sin(A), endPosition[1]],
	# 						  [-np.cos(A)*np.cos(G)*np.sin(B) + np.sin(A)*np.sin(G), np.cos(A)*np.sin(B)*np.sin(G) + np.cos(G)*np.sin(A), np.cos(A)*np.cos(B), endPosition[2]],
	# 						  [0, 0, 0, 1]])

	
	transform0End = np.array([[np.cos(B)*np.cos(G), -np.cos(B)*np.sin(G), np.sin(B), endPosition[0]], 
							  [np.cos(A)*np.sin(G) + np.cos(G)*np.sin(A)*np.sin(B), np.cos(A)*np.cos(G) - np.sin(A)*np.sin(B)*np.sin(G), -np.cos(B)*np.sin(A), endPosition[1]], 
							  [-np.cos(A)*np.cos(G)*np.sin(B) + np.sin(A)*np.sin(G), np.cos(A)*np.sin(B)*np.sin(G) + np.cos(G)*np.sin(A), np.cos(A)*np.cos(B), endPosition[2]],
							  [0, 0, 0, 1]])
	
	# transform0End = np.array([[np.cos(B)*np.cos(G), -np.cos(A)*np.sin(G) + np.cos(G)*np.sin(A)*np.sin(B), np.cos(A)*np.cos(G)*np.sin(B) + np.sin(A)*np.sin(G), endPosition[0]], 
	# 						  [np.cos(B)*np.sin(G), np.cos(A)*np.cos(G) + np.sin(A)*np.sin(B)*np.sin(G),  np.cos(A)*np.sin(B)*np.sin(G) - np.cos(G)*np.sin(A), endPosition[1]], 
	# 						  [-np.sin(B), 			 np.cos(B)*np.sin(A), 											   np.cos(A)*np.cos(B), endPosition[2]],
	# 						  [0, 0, 0, 1]])

	# transform0End = np.array([[-1, 0, 0, endPosition[0]],
	# 						  [0, -1, 0, endPosition[1]],
	# 						  [0, 0, 1, endPosition[2]],
	# 						  [0, 0, 0, 1]])
	transformEndSw = np.array([[1, 0, 0, -endEffector[0]-62],
							   [0, 1, 0, -endEffector[1]],
							   [0, 0, 1, -endEffector[2]],
							   [0, 0, 0, 1]])
	transform0Sw = transform0End @ transformEndSw

	sphericalPos = np.hstack(transform0Sw[:-1,3])
	# print(f'Spherical pos1: {sphericalPos}')
	sphericalPos = np.array(sphericalPos).ravel()

	# print(f'Spherical pos2: {sphericalPos}')
	
	xSw = sphericalPos[0]
	ySw = sphericalPos[1]
	zSw = sphericalPos[2]
 
	hSw = zSw - lengths[0]	
 
	RSw = np.sqrt(xSw**2 + ySw**2)
	rSw = np.sqrt(hSw**2 + RSw**2)
	alpha2 = np.arctan2(hSw, RSw)

	# print(f'alpha2: {alpha2}'s)
	# print(f'RSw: {RSw}, rSw: {rSw}, hSw: {hSw}, alpha2: {alpha2}, xSw, ySw, zSw: {[xSw, ySw, zSw]}')

	theta1 = np.arctan2(ySw, xSw)
	theta2 = -np.arccos((lengths[1]**2 - lengths[2]**2 + rSw**2)/(2*lengths[1]*rSw)) + np.pi/2 - alpha2
	theta3 = -np.arccos((lengths[2]**2 + lengths[1]**2 - rSw**2)/(2*lengths[1]*lengths[2])) + np.pi/2

	theta1 = float(theta1)
	theta2 = float(theta2)
	theta3 = float(theta3)

	rot_mat_0_6 = transform0End[:-1,:-1]

	# check of rot_mat_0_6 is a valid rotation matrix

	m = rot_mat_0_6
	mat_1 = np.sqrt(m[0,0]**2 + m[1,0]**2 + m[2,0]**2)
	mat_2 = np.sqrt(m[0,1]**2 + m[1,1]**2 + m[2,1]**2)
	mat_3 = np.sqrt(m[0,2]**2 + m[1,2]**2 + m[2,2]**2)
	if mat_1 == 1 and mat_2 == 1 and mat_3 == 1:
		print("rot_mat_0_6 is a valid rotation matrix")
	else:
		print("rot_mat_0_6 is NOT a valid rotation matrix")

	# print(rot_mat_0_6)
	# rot_mat_0_3 = np.array([[np.cos(theta1)*np.cos(theta2)*np.cos(theta3) - np.cos(theta1)*np.sin(theta2)*np.sin(theta3), 	-np.sin(theta1), 	np.cos(theta1)*np.cos(theta2)*np.sin(theta3) + np.cos(theta1)*np.cos(theta3)*np.sin(theta2)], 
	# 						[np.cos(theta2)*np.cos(theta3)*np.sin(theta1) - np.sin(theta1)*np.sin(theta2)*np.sin(theta3), 	np.cos(theta1), 	np.cos(theta2)*np.sin(theta1)*np.sin(theta3) + np.cos(theta3)*np.sin(theta1)*np.sin(theta2)], 
	# 						[-np.cos(theta2)*np.sin(theta3) - np.cos(theta3)*np.sin(theta2), 								0, 					np.cos(theta2)*np.cos(theta3) - np.sin(theta2)*np.sin(theta3)]])

	rot_mat_0_3 = np.array([[np.cos(theta1)*np.cos(theta2)*np.cos(theta3) - np.cos(theta1)*np.sin(theta2)*np.sin(theta3), -np.sin(theta1), np.cos(theta1)*np.cos(theta2)*np.sin(theta3) + np.cos(theta1)*np.cos(theta3)*np.sin(theta2)], 
							[np.cos(theta2)*np.cos(theta3)*np.sin(theta1) - np.sin(theta1)*np.sin(theta2)*np.sin(theta3), np.cos(theta1), np.cos(theta2)*np.sin(theta1)*np.sin(theta3) + np.cos(theta3)*np.sin(theta1)*np.sin(theta2)], 
							[-np.cos(theta2)*np.sin(theta3) - np.cos(theta3)*np.sin(theta2), 0, np.cos(theta2)*np.cos(theta3) - np.sin(theta2)*np.sin(theta3)]])

	inv_rot_mat_0_3 = np.linalg.inv(rot_mat_0_3)
	rot_mat_3_6 = inv_rot_mat_0_3 @ rot_mat_0_6

	theta5 = np.arccos(rot_mat_3_6[0,0])
	
	theta6_1 = np.arcsin(rot_mat_3_6[0,1]/np.sin(theta5))
	theta6_2 = np.pi - np.arcsin(rot_mat_3_6[0,1]/np.sin(theta5))

	theta4_1 = np.arcsin(rot_mat_3_6[1,0]/np.sin(theta5))
	theta4_2 = np.pi - np.arcsin(rot_mat_3_6[1,0]/np.sin(theta5))

	# r11 = np.cos(theta5)
	# r12 = np.sin(theta5)*np.sin(theta6)
	# r13 = np.cos(theta6)*np.sin(theta5)
	# r21 = np.sin(theta4)*np.sin(theta5)
	# r22 = np.cos(theta4)*np.cos(theta6) - np.cos(theta5)*np.sin(theta4)*np.sin(theta6)
	# r23 = -np.cos(theta4)*np.sin(theta6) - np.cos(theta5)*np.cos(theta6)*np.sin(theta4)
	# r31 = -np.cos(theta4)*np.sin(theta5)
	# r32 = np.cos(theta4)*np.cos(theta5)*np.sin(theta6) + np.cos(theta6)*np.sin(theta4)
	# r33 = np.cos(theta4)*np.cos(theta5)*np.cos(theta6) - np.sin(theta4)*np.sin(theta6)

	# check_rot_mat_3_6 = np.array([[r11, r12, r23],
	# 							  [r21, r22, r23],
	# 							  [r31, r32, r33]])

	# rot_minus_check_3_6 = rot_mat_3_6 - check_rot_mat_3_6
	# print(f'rot_minus_check_3_6: {rot_minus_check_3_6}')
	# rot_minus_check_3_6 = rot_mat_3_6.round() - check_rot_mat_3_6.round()

	# zero_matrix = np.array([[0, 0, 0],
	# 						[0, 0, 0],
	# 						[0, 0, 0]])
	# matrices_are_equal = np.array_equal(rot_minus_check_3_6, zero_matrix)
	

	# print(f'rot_mat_3_6 {rot_mat_3_6}')
	# print(f'check_rot_mat_3_6 {check_rot_mat_3_6}')
	# print(f'Matrices are equal?: {matrices_are_equal}')
	# # r06
	# # c36
	# # r03
	# # r36 = r03^-1 @ r06

		
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
	
	# calculate tool positions for both possible angles to see which angle produces a closer result
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

	distFrom1 = np.sqrt((toolpos1[0] - endPosition[0])**2 + (toolpos1[1] - endPosition[1])**2 + (toolpos1[2] - endPosition[2])**2)
	distFrom2 = np.sqrt((toolpos2[0] - endPosition[0])**2 + (toolpos2[1] - endPosition[1])**2 + (toolpos2[2] - endPosition[2])**2)

	theta4 = 0
	theta6 = 0
	# choose theta 4 and 6 pair that results in least distance between desired and calculated end effector position
	if distFrom1 < distFrom2:
		theta4 = theta4_1
		theta6 = theta6_1
		baseTransforms = baseTransforms1
	else:
		theta4 = theta4_2
		theta6 = theta6_2
		baseTransforms = baseTransforms2
		
	print(f'theta123: {[theta1, theta2, theta3, theta4, theta5, theta6]}')

	# Extract position and rotation data for each joint
	jointPositions = np.array([baseTransforms[i,:-1,3] for i in range(0, baseTransforms.shape[0])])
	jointRotationMatrices = np.array([baseTransforms[i,:-1,:-1] for i in range(0, baseTransforms.shape[0])])

	ax.clear()
	ax.scatter([0, 0], [0, 0], [maxZ, minZ], s=0)  # Keeps z axis constant
	ax.axis([-400, 400, -400, 400])

	# print(f'BASEtRANSFRMS: {baseTransforms}')
	# print(f'jointPositions: {jointPositions}')
	
	ax.scatter(sphericalPos[0], sphericalPos[1], sphericalPos[2], color='.6')
	ax.scatter(0, 0, 0, color='.3')
	ax.scatter(endPosition[0], endPosition[1], endPosition[2], color='.3')
	[ax.scatter(jointPositions[i,0], jointPositions[i,1], jointPositions[i,2], color='.6') for i in range(0,4)]
	
	xline = [sphericalPos[0], endPosition[0]]
	yline = [sphericalPos[1], endPosition[1]]
	zline = [sphericalPos[2], endPosition[2]]

	ax.plot3D(xline, yline, zline, 'chartreuse')
	# [ax.plot3D((jointPositions[i,0], jointPositions[i+1,0]), (jointPositions[i,1], jointPositions[i+1,1]), (jointPositions[i,2], jointPositions[i+1,2])) for i in range(0,2)]
	xline0 = [0, jointPositions[0,0]]
	yline0 = [0, jointPositions[0,1]]
	zline0 = [0, jointPositions[0,2]]
	ax.plot3D(xline0, yline0, zline0)
	xline1 = [jointPositions[0,0], jointPositions[1,0]]
	yline1 = [jointPositions[0,1], jointPositions[1,1]]
	zline1 = [jointPositions[0,2], jointPositions[1,2]]
	ax.plot3D(xline1, yline1, zline1)
	xline2 = [jointPositions[1,0], jointPositions[2,0]]
	yline2 = [jointPositions[1,1], jointPositions[2,1]]
	zline2 = [jointPositions[1,2], jointPositions[2,2]]
	ax.plot3D(xline2, yline2, zline2)
	xline3 = [jointPositions[2,0], jointPositions[3,0]]
	yline3 = [jointPositions[2,1], jointPositions[3,1]]
	zline3 = [jointPositions[2,2], jointPositions[3,2]]
	ax.plot3D(xline3, yline3, zline3)
	xline3 = [jointPositions[3,0], jointPositions[4,0]]
	yline3 = [jointPositions[3,1], jointPositions[4,1]]
	zline3 = [jointPositions[3,2], jointPositions[4,2]]
	ax.plot3D(xline3, yline3, zline3)
	xline3 = [jointPositions[4,0], jointPositions[5,0]]
	yline3 = [jointPositions[4,1], jointPositions[5,1]]
	zline3 = [jointPositions[4,2], jointPositions[5,2]]
	ax.plot3D(xline3, yline3, zline3)
	xline3 = [jointPositions[5,0], jointPositions[6,0]]
	yline3 = [jointPositions[5,1], jointPositions[6,1]]
	zline3 = [jointPositions[5,2], jointPositions[6,2]]
	ax.plot3D(xline3, yline3, zline3)

	ax.text(0, 0, 0, 0, color='orange')
	for i in range(0, 6):
		ax.text(jointPositions[i,0], jointPositions[i,1], jointPositions[i,2], i+1, color='orange')
	ax.text(endPosition[0], endPosition[1], endPosition[2], "tool", color='orange')
 
 
	axesXYZsGlobal = np.array([[50, 0, 0],[0, 50, 0],[0, 0, 50]])  # global xyz coordinates
	axesXYZsGlobalMatrix = np.array([[[1, 0, 0, axesXYZsGlobal[i,0]],  # make matrix out of global xyz coords
						  	 		  [0, 1, 0, axesXYZsGlobal[i,1]],
						  			  [0, 0, 1, axesXYZsGlobal[i,2]],
						  	 		  [0, 0, 0, 1]] for i in range(0, axesXYZsGlobal.shape[0])])
	locations = np.array([transform0Sw, transform0End, transform01, transform02, transform03, baseTransforms[3], baseTransforms[4], baseTransforms[5], baseTransforms[6]])
	# locations = np.array([transform0Sw, transform0End])
	axesXYZsLocalMatrix = [locations[t] @ axesXYZsGlobalMatrix[i]  for t in range(0, locations.shape[0]) for i in range(0, axesXYZsGlobal.shape[0])]  # transform into each joint's coords
	# print(axesXYZsLocalMatrix)
	axesXYZsLocalMatrix = np.reshape(axesXYZsLocalMatrix, (27, 4, -1))  # reshape for usability
	axesXYZsLocal = np.array([axesXYZsLocalMatrix[i,:-1,3] for i in range(0, axesXYZsLocalMatrix.shape[0])])  # extract position data from matrix
 
	localXAxes = axesXYZsLocal[0::3]
	localYAxes = axesXYZsLocal[1::3]
	localZAxes = axesXYZsLocal[2::3]
 
	jointPositions2 = np.array([sphericalPos, endPosition, jointPositions[0], jointPositions[1], jointPositions[2]])
 
	[ax.plot3D((jointPositions2[i,0], localXAxes[i,0]),(jointPositions2[i,1], localXAxes[i,1]),(jointPositions2[i,2], localXAxes[i,2]), 'red') for i in range(0,5)]
	[ax.plot3D((jointPositions2[i,0], localYAxes[i,0]),(jointPositions2[i,1], localYAxes[i,1]),(jointPositions2[i,2], localYAxes[i,2]), 'green') for i in range(0,5)]
	[ax.plot3D((jointPositions2[i,0], localZAxes[i,0]),(jointPositions2[i,1], localZAxes[i,1]),(jointPositions2[i,2], localZAxes[i,2]), 'blue') for i in range(0,5)]
 
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
sliderXend = Slider(xend, 'xend', -800, 800, valinit=endPosition[0])
sliderYend  = Slider(yend, 'yend', -800, 800, valinit=endPosition[1])
sliderZend = Slider(zend, 'zend', -400, 800, valinit=endPosition[2])
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
