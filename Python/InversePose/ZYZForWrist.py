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

wheelForward = sym.Matrix([[sina1, sina2, sina3],
				   [cosa1, cosa2, cosa3],
				   [1, 1, 1]])
# wheelInv = wheelForward.inv()
# print(wheelInv)

alpha1 = 5
alpha2 = 3
alpha3 = 1

wheelInvMatrix = np.array([[-(cos(alpha2) - cos(alpha3))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), (-(-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) + (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)) - (cos(alpha2) - cos(alpha3))*(sin(alpha1) - sin(alpha2)))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)))), (cos(alpha2)*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))) - (cos(alpha2) - cos(alpha3))*(cos(alpha1)*(-sin(alpha1) + sin(alpha2)) - sin(alpha1)*(-cos(alpha1) + cos(alpha2))))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))))],
[-(-cos(alpha1) + cos(alpha3))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), ((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)) - (-cos(alpha1) + cos(alpha3))*(sin(alpha1) - sin(alpha2)))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)))), (-cos(alpha1)*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))) - (-cos(alpha1) + cos(alpha3))*(cos(alpha1)*(-sin(alpha1) + sin(alpha2)) - sin(alpha1)*(-cos(alpha1) + cos(alpha2))))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))))],
[(-cos(alpha1) + cos(alpha2))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), (sin(alpha1) - sin(alpha2))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), (cos(alpha1)*(-sin(alpha1) + sin(alpha2)) - sin(alpha1)*(-cos(alpha1) + cos(alpha2)))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)))]])

print(wheelInvMatrix)
