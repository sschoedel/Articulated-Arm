import sympy as sym
from sympy import *
import numpy as np
from math import sin, cos
sina1 = sym.Symbol('sin(alpha1)')
sina2 = sym.Symbol('sin(alpha2)')
sina3 = sym.Symbol('sin(alpha3)')
cosa1 = sym.Symbol('cos(alpha1)')
cosa2 = sym.Symbol('cos(alpha2)')
cosa3 = sym.Symbol('cos(alpha3)')

# wheelForward = sym.Matrix([[sina1, sina2, sina3],
# 				   [cosa1, cosa2, cosa3],
# 				   [1, 1, 1]])
# # wheelInv = wheelForward.inv()
# # print(wheelInv)

# alpha1 = 5
# alpha2 = 3
# alpha3 = 1

# wheelInvMatrix = np.array([[-(cos(alpha2) - cos(alpha3))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), (-(-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) + (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)) - (cos(alpha2) - cos(alpha3))*(sin(alpha1) - sin(alpha2)))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)))), (cos(alpha2)*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))) - (cos(alpha2) - cos(alpha3))*(cos(alpha1)*(-sin(alpha1) + sin(alpha2)) - sin(alpha1)*(-cos(alpha1) + cos(alpha2))))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))))],
# [-(-cos(alpha1) + cos(alpha3))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), ((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)) - (-cos(alpha1) + cos(alpha3))*(sin(alpha1) - sin(alpha2)))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)))), (-cos(alpha1)*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))) - (-cos(alpha1) + cos(alpha3))*(cos(alpha1)*(-sin(alpha1) + sin(alpha2)) - sin(alpha1)*(-cos(alpha1) + cos(alpha2))))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))))],
# [(-cos(alpha1) + cos(alpha2))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), (sin(alpha1) - sin(alpha2))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), (cos(alpha1)*(-sin(alpha1) + sin(alpha2)) - sin(alpha1)*(-cos(alpha1) + cos(alpha2)))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)))]])

# print(wheelInvMatrix)


sint0 = sym.Symbol('sin(theta[0])')
sint1 = sym.Symbol('sin(theta[1])')
sint2 = sym.Symbol('sin(theta[2])')
sint3 = sym.Symbol('sin(theta[3])')
sint4 = sym.Symbol('sin(theta[4])')
sint5 = sym.Symbol('sin(theta[5])')
cost0 = sym.Symbol('cos(theta[0])')
cost1 = sym.Symbol('cos(theta[1])')
cost2 = sym.Symbol('cos(theta[2])')
cost3 = sym.Symbol('cos(theta[3])')
cost4 = sym.Symbol('cos(theta[4])')
cost5 = sym.Symbol('cos(theta[5])')

# YZY wrist rotation matrix (used in forward kinematics)
t34 = np.array([[cost3, 0, sint3],
                [0, 1, 0],
                [-sint3, 0, cost3]])
t45 = np.array([[cost4, -sint4, 0],
                [sint4, cost4, 0],
                [0, 0, 1]])
t56 = np.array([[cost5, 0, sint5],
                [0, 1, 0],
                [-sint5, 0, cost5]])

c36 = t34 @ t45 @ t56

# these are the variables we're solving for (theta4, 5, 6)
print(c36) # c36 equation
# [[cos(theta4)*cos(theta5)*cos(theta6) - sin(theta4)*sin(theta6),  -cos(theta4)*sin(theta5), cos(theta4)*cos(theta5)*sin(theta6) + cos(theta6)*sin(theta4)],
#  [cos(theta6)*sin(theta5), 									    cos(theta5),			  sin(theta5)*sin(theta6)],
#  [-cos(theta4)*sin(theta6) - cos(theta5)*cos(theta6)*sin(theta4), sin(theta4)*sin(theta5),  cos(theta4)*cos(theta6) - cos(theta5)*sin(theta4)*sin(theta6)]]

# theta5 = np.arccos(rot_mat_3_6[1][1])
# theta4 = -np.arccos(rot_mat_3_6[0][1]/np.sin(theta5))
# theta6 = np.arccos(rot_mat_3_6[1][0]/np.sin(theta5))


# these are the variables we know (alpha, beta, gamma, theta1, theta2, theta3)
# XYZ rotation matrix for end effector

x = np.array([[1, 0, 0],
                [0, cost3, -sint3],
                [0, sint3, cost3]])
y = np.array([[cost4, 0, sint4],
                [0, 1, 0],
                [-sint4, 0, cost4]])
z = np.array([[cost5, -sint5, 0],
                [sint5, cost5, 0],
                [0, 0, 1]])
print("")
r06 = x @ y @ z
print(r06)

# [[cos(beta)*cos(gamma), -cos(beta)*sin(gamma), sin(beta)],
#  [cos(alpha)*sin(gamma) + cos(gamma)*sin(alpha)*sin(beta), cos(alpha)*cos(gamma) - sin(alpha)*sin(beta)*sin(gamma), -cos(beta)*sin(alpha)],
#  [-cos(alpha)*cos(gamma)*sin(beta) + sin(alpha)*sin(gamma), cos(alpha)*sin(beta)*sin(gamma) + cos(gamma)*sin(alpha), cos(alpha)*cos(beta)]]


# r36 calculated by r36 = (r03)^-1 @ r06


# r03
r01 = np.array([[cost0, -sint0, 0],
				[sint0, cost0, 0],
				[0, 0, 1]])
r12 = np.array([[cost1 - np.pi/2, -sint1 - np.pi/2, 0],
				[0, 0, 1],
				[-sint1 - np.pi/2, -cost1 - np.pi/2, 0]])
r23 = np.array([[cost2, -sint2, 0],
				[sint2, cost2, 0],
				[0, 0, 1]])

r03 = r01 @ r12 @ r23
print("")
print(r03)

# np.array([[cos(theta[0])*cos(theta[2])*(cos(theta[1]) - np.pi/2) + cos(theta[0])*sin(theta[2])*(-sin(theta[1]) - np.pi/2), cos(theta[0])*cos(theta[2])*(-sin(theta[1]) - np.pi/2) - cos(theta[0])*sin(theta[2])*(cos(theta[1]) - np.pi/2), -sin(theta[0])],
# 		  [cos(theta[2])*sin(theta[0])*(cos(theta[1]) - np.pi/2) + sin(theta[0])*sin(theta[2])*(-sin(theta[1]) - np.pi/2), cos(theta[2])*sin(theta[0])*(-sin(theta[1]) - np.pi/2) - sin(theta[0])*sin(theta[2])*(cos(theta[1]) - np.pi/2), cos(theta[0])],
# 		  [cos(theta[2])*(-sin(theta[1]) - np.pi/2) + sin(theta[2])*(-cos(theta[1]) - np.pi/2), cos(theta[2])*(-cos(theta[1]) - np.pi/2) - sin(theta[2])*(-sin(theta[1]) - np.pi/2), 0]])