import sympy as sym
from sympy import *
import numpy as np

# sinTheta0 = sym.Symbol('sin(theta[0])')
# sinTheta1 = sym.Symbol('sin(theta[1])')
# sinTheta2 = sym.Symbol('sin(theta[2])')
# sinTheta3 = sym.Symbol('sin(theta[3])')
# sinTheta4 = sym.Symbol('sin(theta[4])')
# sinTheta5 = sym.Symbol('sin(theta[5])')

# cosTheta0 = sym.Symbol('cos(theta[0])')
# cosTheta1 = sym.Symbol('cos(theta[1])')
# cosTheta2 = sym.Symbol('cos(theta[2])')
# cosTheta3 = sym.Symbol('cos(theta[3])')
# cosTheta4 = sym.Symbol('cos(theta[4])')
# cosTheta5 = sym.Symbol('cos(theta[5])')

sinTheta0 = sym.Symbol('sT0')
sinTheta1 = sym.Symbol('sT1')
sinTheta2 = sym.Symbol('sT2')
sinTheta3 = sym.Symbol('sT3')
sinTheta4 = sym.Symbol('sT4')
sinTheta5 = sym.Symbol('sT5')

cosTheta0 = sym.Symbol('cT0')
cosTheta1 = sym.Symbol('cT1')
cosTheta2 = sym.Symbol('cT2')
cosTheta3 = sym.Symbol('cT3')
cosTheta4 = sym.Symbol('cT4')
cosTheta5 = sym.Symbol('cT5')

xOff0 = sym.Symbol('xOff0')
xOff1 = sym.Symbol('xOff1')
xOff2 = sym.Symbol('xOff2')
xOff3 = sym.Symbol('xOff3')
xOff4 = sym.Symbol('xOff4')
xOff5 = sym.Symbol('xOff5')
xTpos = sym.Symbol('xTpos')

yOff0 = sym.Symbol('yOff0')
yOff1 = sym.Symbol('yOff1')
yOff2 = sym.Symbol('yOff2')
yOff3 = sym.Symbol('yOff3')
yOff4 = sym.Symbol('yOff4')
yOff5 = sym.Symbol('yOff5')
yTpos = sym.Symbol('yTpos')

zOff0 = sym.Symbol('zOff0')
zOff1 = sym.Symbol('zOff1')
zOff2 = sym.Symbol('zOff2')
zOff3 = sym.Symbol('zOff3')
zOff4 = sym.Symbol('zOff4')
zOff5 = sym.Symbol('zOff5')
zTpos = sym.Symbol('zTpos')

transform01 = sym.Matrix([[cosTheta0, -sinTheta0, 0, xOff0], 
						[sinTheta0, cosTheta0, 0, yOff0], 
						[0, 0, 1, zOff0], 
						[0, 0, 0, 1]])
transform12 = sym.Matrix([[cosTheta1, -sinTheta1, 0, xOff1],
						[0, 0, 1, yOff1], 
						[-sinTheta1, -cosTheta1, 0, zOff1], 
						[0, 0, 0, 1]])
transform23 = sym.Matrix([[cosTheta2, -sinTheta2, 0, xOff2], 
						[sinTheta2, cosTheta2, 0, yOff2], 
						[0, 0, 1, zOff2], 
						[0, 0, 0, 1]])
transform34 = sym.Matrix([[cosTheta3, -cosTheta3, 0, xOff3], 
						[0, 0, 1, yOff3], 
						[-sinTheta3, -cosTheta3, 0, zOff3], 
						[0, 0, 0, 1]])
transform45 = sym.Matrix([[cosTheta4, -sinTheta4, 0, xOff4], 
						[0, 0, 1, yOff4], 
						[-sinTheta4, -cosTheta4, 0, zOff4], 
						[0, 0, 0, 1]])
transform56 = sym.Matrix([[cosTheta5, -sinTheta5, 0, xOff5], 
						[0, 0, -1, yOff5], 
						[sinTheta5, cosTheta5, 0, zOff5], 
						[0, 0, 0, 1]])
transform6Tool = sym.Matrix([[1, 0, 0, xTpos],
							[0, 1, 0, yTpos],
							[0, 0, 1, zTpos],
							[0, 0, 0, 1]])

# Mult matrices together to get transformation matrix from frame 0 to frame x
transform06 = transform01 * transform12 * transform23 * transform34 * transform45 * transform56
transform0Tool = transform06 * transform6Tool

inverseTool = transform6Tool**-1
print(inverseTool)

finalT = str(transform0Tool)
transforms = finalT.split(',')

# for t in transforms:
# 	print(t)
# 	print("")