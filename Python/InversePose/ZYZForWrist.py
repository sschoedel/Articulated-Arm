import sympy as sym
from sympy import *
import numpy as np

cosT4 = sym.Symbol('cT4')
cosbeta = sym.Symbol('cbeta')
cosT6 = sym.Symbol('cT6')
sinT4 = sym.Symbol('sT4')
sinbeta = sym.Symbol('sbeta')
sinT6 = sym.Symbol('sT6')

zRot1 = sym.Matrix([[cosT4, -sinT4, 0],
				   [sinT4, cosT4, 0],
				   [0, 0, 1]])
yRot = sym.Matrix([[cosbeta, 0, sinbeta],
				   [0, 1, 0],
				   [-sinbeta, 0, cosbeta]])
zRot2 = sym.Matrix([[cosT6, -sinT6, 0],
				   [sinT6, cosT6, 0],
				   [0, 0, 1]])

zyz = zRot1 * yRot * zRot2

# t4 = np.arctan2(sym.sqrt(1-cosbeta**2), cosbeta)  # plus or minus
# t5 = np.arctan2(sinalpha*sinbeta, cosalpha*sinbeta)
# t6 = np.arctan2(sinbeta*singamma, -cosgamma*sinbeta)

print(zyz)