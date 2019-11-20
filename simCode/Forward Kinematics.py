import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

# Theta inputs in degrees
z1 = 20
z2 = -40
z3 = 20
z4 = 30
z5 = 50
z6 = 130
# z1 = 0
# z2=0
# z3=0
# z4=0
# z5=0
# z6=0

theta = np.array([z1 * 180/np.pi, z2 * 180/np.pi, z3 * 180/np.pi, z4 * 180/np.pi, z5 * 180/np.pi, z6 * 180/np.pi])
alpha = np.array([0, -np.pi/2, 0, -np.pi/2, -np.pi/2, np.pi/2])

toolPosition = np.array([0, 0, 60])

xOff = np.array([22.12, 0, 300.32, 0, 0, 0])
yOff = np.array([0, 0, 0, 86.3, 0, 0])
zOff = np.array([135.7, 31.8, 0, 293, 0, 62])

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
transform12 = np.array([[np.cos(theta[1] + np.pi/2), -np.sin(theta[1] + np.pi/2), 0, xOff[1]],
						[0, 0, 1, yOff[1]], 
						[-np.sin(theta[1] + np.pi/2), -np.cos(theta[1] + np.pi/2), 0, zOff[1]], 
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
transform6Tool = np.array([[1, 0, 0, 0],
						   [0, 1, 0, 0],
						   [0, 0, 1, toolPosition[2]],
						   [0, 0, 0, 1]])

transform = np.array([transform01, transform12, transform23, transform34, transform45, transform56, transform6Tool])

# Mult all arrays together to get transformation matrix from frame 0 to frame x
transform02 = transform[0] @ transform[1]
transform03 = transform[0] @ transform[1] @ transform[2]
transform04 = transform[0] @ transform[1] @ transform[2] @ transform[3]
transform05 = transform[0] @ transform[1] @ transform[2] @ transform[3] @ transform[4]
transform06 = transform[0] @ transform[1] @ transform[2] @ transform[3] @ transform[4] @ transform[5]
toolPos06 = transform06 @ transform6Tool



# beta = np.atan2(-r31, +-np.sqrt(r11** + r21**))
# alpha = np.atan2(r21/cos(beta), r11/cos(beta))
# yeta = np.atan2(r32/cos(beta), r33/cos(beta))

beta = np.arctan2(-toolPos06[2][0], np.sqrt(toolPos06[0][0]**2 + toolPos06[1][0]**2))  # z rotation
alpha = np.arctan2(toolPos06[1][0]/np.cos(beta), toolPos06[0][0]/np.cos(beta))  # y rotation
yeta = np.arctan2(toolPos06[2][1]/np.cos(beta), toolPos06[2][2]/np.cos(beta))  # x rotation

np.set_printoptions(linewidth=100, precision=2, suppress=True)
print("transform from frame 0 to 6")
print(transform06)

print("")

print("position of tool in frame 0")
print(toolPos06)

print("")

print("rotation of tool in frame 0 in degrees")
print("X     				Y     				 Z")
print("{}   {}    {}".format(yeta * 180/np.pi, alpha * 180/np.pi, beta * 180/np.pi))