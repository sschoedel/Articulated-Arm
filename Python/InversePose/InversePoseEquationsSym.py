# Gives three position and three rotation values for end
# effector and solves for joint angles.

# Inputs: three positions three rotations for end effector - fully defined
# 			in = (x, y, z, alpha, beta, gamma)
# Outputs: one equation for each joint angle in terms of previous joint angles
# 			t1 = f(in)
# 			t2 = f(in, t1)
# 			t3 = f(in, t1, t2)
# 			t4 = f(in, t1, t2, t3)
# 			t5 = f(in, t1, t2, t3, t4)
# 			t6 = f(in, t1, t2, t3, t4, t5)

import numpy as np
import sympy as sym

# Specified end effector positions
xEnd = sym.Symbol('xEnd')
yEnd = sym.Symbol('yEnd')
zEnd = sym.Symbol('zEnd')

# Specified end effector rotations
aEnd = sym.Symbol('aEnd')
bEnd = sym.Symbol('bEnd')
gEnd = sym.Symbol('gEnd')

# Known values
