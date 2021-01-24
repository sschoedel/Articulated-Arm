import sympy as sym
from sympy import *
import numpy as np
from math import sin, cos

sint1 = sym.Symbol('sin(theta1)')
sint2 = sym.Symbol('sin(theta2)')
sint3 = sym.Symbol('sin(theta3)')
sint4 = sym.Symbol('sin(theta4)')
sint5 = sym.Symbol('sin(theta5)')
sint6 = sym.Symbol('sin(theta6)')
cost1 = sym.Symbol('cos(theta1)')
cost2 = sym.Symbol('cos(theta2)')
cost3 = sym.Symbol('cos(theta3)')
cost4 = sym.Symbol('cos(theta4)')
cost5 = sym.Symbol('cos(theta5)')
cost6 = sym.Symbol('cos(theta6)')

sinA = sym.Symbol('sin(A)')
sinB = sym.Symbol('sin(B)')
sinG = sym.Symbol('sin(G)')
cosA = sym.Symbol('cos(A)')
cosB = sym.Symbol('cos(B)')
cosG = sym.Symbol('cos(G)')

# XYX wrist rotation matrix (used in forward kinematics)
c34 = sym.Matrix([[1, 0, 0], 
				[0, cost4, -sint4], 
				[0, sint4, cost4]]) 
c45 = sym.Matrix([[cost5, 0, sint5], 
				[0, 1, 0], 
				[-sint5, 0, cost5]])
c56 = sym.Matrix([[1, 0, 0],
				[0, cost6, -sint6],
				[0, sint6, cost6]])

c36 = c34 @ c45 @ c56

# these are the variables we're solving for (theta4, 5, 6)
print("c36: ")
print(c36) # c36 equation

# Matrix([[cos(theta5), sin(theta5)*sin(theta6), cos(theta6)*sin(theta5)], 
# 		  [sin(theta4)*sin(theta5), cos(theta4)*cos(theta6) - cos(theta5)*sin(theta4)*sin(theta6), -cos(theta4)*sin(theta6) - cos(theta5)*cos(theta6)*sin(theta4)], 
# 		  [-cos(theta4)*sin(theta5), cos(theta4)*cos(theta5)*sin(theta6) + cos(theta6)*sin(theta4), cos(theta4)*cos(theta5)*cos(theta6) - sin(theta4)*sin(theta6)]])

# theta5 = np.arccos(rot_mat_3_6[0][0])
# theta6 = np.arcsin(rot_mat_3_6[0][1]/np.sin(theta5))
# theta4 = np.arcsin(rot_mat_3_6[1][0]/np.sin(theta5))


# these are the variables we know (alpha, beta, gamma, theta1, theta2, theta3)
# XYZ rotation matrix for end effector

x = sym.Matrix([[1, 0, 0],
			[0, cosA, -sinA],
			[0, sinA, cosA]])
y = sym.Matrix([[cosB, 0, sinB],
			[0, 1, 0],
			[-sinB, 0, cosB]])
z = sym.Matrix([[cosG, -sinG, 0],
			[sinG, cosG, 0],
			[0, 0, 1]])
print("r06: ")
r06 = x @ y @ z
print(r06)

# x@y@z
# ([[cos(B)*cos(G), -cos(B)*sin(G), sin(B)], 
# [cos(A)*sin(G) + cos(G)*sin(A)*sin(B), cos(A)*cos(G) - sin(A)*sin(B)*sin(G), -cos(B)*sin(A)], 
# [-cos(A)*cos(G)*sin(B) + sin(A)*sin(G), cos(A)*sin(B)*sin(G) + cos(G)*sin(A), cos(A)*cos(B)]])


# r36 calculated by r36 = (r03)^-1 @ r06

# r03
r01 = sym.Matrix([[cost1, -sint1, 0], 
				[sint1, cost1, 0], 
				[0, 0, 1]])
r12 = sym.Matrix([[cost2, 0, sint2],
				[0, 1, 0],
				[-sint2, 0, cost2]])
r23 = sym.Matrix([[cost3, 0, sint3],
				[0, 1, 0],
				[-sint3, 0, cost3]]) 

r03 = r01 @ r12 @ r23
print("r03: ")
print(r03)

# [[cos(theta[0])*cos(theta[1])*cos(theta[2]) - cos(theta[0])*sin(theta[1])*sin(theta[2]), 	-sin(theta[0]), cos(theta[0])*cos(theta[1])*sin(theta[2]) + cos(theta[0])*cos(theta[2])*sin(theta[1])], 
#  [cos(theta[1])*cos(theta[2])*sin(theta[0]) - sin(theta[0])*sin(theta[1])*sin(theta[2]), 	cos(theta[0]), 	cos(theta[1])*sin(theta[0])*sin(theta[2]) + cos(theta[2])*sin(theta[0])*sin(theta[1])], 
#  [-cos(theta[1])*sin(theta[2]) - cos(theta[2])*sin(theta[1]), 								0, 			cos(theta[1])*cos(theta[2]) - sin(theta[1])*sin(theta[2])]

# [[cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3), -sin(theta1), cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)], 
#  [cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3), cos(theta1), cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)], 
#  [-cos(theta2)*sin(theta3) - cos(theta3)*sin(theta2), 0, cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)]]