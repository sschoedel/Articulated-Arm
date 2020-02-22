import sympy as sym
from sympy import *
import numpy as np
import math

xIn = sym.Symbol('x')
yIn = sym.Symbol('y')
zIn = sym.Symbol('z')
cosa = sym.Symbol('np.cos(A)')
cosb = sym.Symbol('np.cos(B)')
cosg = sym.Symbol('np.cos(G)')
sina = sym.Symbol('np.sin(A)')
sinb = sym.Symbol('np.sin(B)')
sing = sym.Symbol('np.sin(G)')
lengthEnd = sym.Symbol('lenEnd')

endEffector = sym.Matrix([0, 0, lengthEnd])
endPosition = sym.Matrix([xIn, yIn, zIn])

xRot = sym.Matrix([[1, 0, 0],
				   [0, cosa, -sina],
				   [0, sina, cosa]])
yRot = sym.Matrix([[cosb, 0, sinb],
				   [0, 1, 0],
				   [-sinb, 0, cosb]])
zRot = sym.Matrix([[cosg, -sing, 0],
				   [sing, cosg, 0],
				   [0, 0, 1]])

rotmat = xRot * yRot * zRot
transform0EndSym = rotmat.col_insert(3, endPosition).row_insert(3, sym.Matrix([[0, 0, 0, 1]]))
transformEndSwSym = sym.Matrix([[1, 0, 0, 0],
							[0, 1, 0, 0],
							[0, 0, 1, -lengthEnd],
							[0, 0, 0, 1]])

transform0SwSym = transform0EndSym * transformEndSwSym
print(transform0SwSym)
print("")
lenEnd = 60
x = 400
y = 350
z = 500
A = 10
B = 0
G = 0
transform0End = np.array([[np.cos(B)*np.cos(G), -np.cos(B)*np.sin(G), np.sin(B), x],
						   [np.cos(A)*np.sin(G) + np.cos(G)*np.sin(A)*np.sin(B), np.cos(A)*np.cos(G) - np.sin(A)*np.sin(B)*np.sin(G), -np.cos(B)*np.sin(A), y],
						   [-np.cos(A)*np.cos(G)*np.sin(B) + np.sin(A)*np.sin(G), np.cos(A)*np.sin(B)*np.sin(G) + np.cos(G)*np.sin(A), np.cos(A)*np.cos(B), z],
						   [0, 0, 0, 1]])
transformEndSw = np.array([[1, 0, 0, 0],
						  [0, 1, 0, 0],
						  [0, 0, 1, -lenEnd],
						  [0, 0, 0, 1]])
transform0Sw = transform0End @ transformEndSw

print(transform0Sw)
sphericalPos = np.vstack(transform0Sw[:-1,3])
print(sphericalPos)