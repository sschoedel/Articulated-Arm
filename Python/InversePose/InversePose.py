import numpy as np
import math

# inputs: xEnd, yEnd, zEnd, alpha, beta, gamma

class Vector(object):
	def __init__(self, xEnd, yEnd, zEnd, xBegin, yBegin, zBegin, length):
		self.distance = length
		self.xDir = (xEnd - xBegin)/self.distance
		self.yDir = (yEnd - yBegin)/self.distance
		self.zDir = (zEnd - zBegin)/self.distance
	def get_direction(self):
		return [self.xDir, self.yDir, self.zDir]

lenEnd = 60
lengths = np.array([135.7, 300.32, 293])
# back up from end position to spherical joint position
def invPose(xEnd, yEnd, zEnd, alpha, beta, gamma):
	xSw = -lenEnd*np.sin(beta) + xEnd
	ySw = lenEnd*np.cos(beta)*np.sin(alpha) + yEnd
	zSw = -lenEnd*np.cos(alpha)*np.cos(beta) + zEnd
	hSw = zSw - lengths[0]
	if np.sqrt(xSw**2 + ySw**2 + hSw**2) > sum(lengths[1:]):
		print("Desired position and orientation not in workspace.")
	else:
		RSw = np.sqrt(xSw**2 + ySw**2)
		rSw = np.sqrt(hSw**2 + RSw**2)
		alpha2 = np.arcsin(hSw/rSw)

		# First three joint angles responsible for placing end effector
		# Using law of cosines:
		theta1 = np.arctan(ySw/xSw)
		theta2 = np.arccos((lengths[1]**2 - lengths[2]**2 + rSw**2)/(2*lengths[1]*rSw)) + alpha2
		theta3 = np.arccos((lengths[2]**2 + lengths[1]**2 - rSw**2)/(2*lengths[1]*lengths[2]))
		# Final three joint angles specify rotation only
		# Multi stuff:
		dirEnd = Vector(xEnd, yEnd, zEnd, xSw, ySw, zSw, lenEnd)
		rElbow = lengths[1] * math.cos(theta2)
		xElbow = rElbow * math.cos(theta1)
		yElbow = rElbow * math.sin(theta1)
		zElbow = lengths[0] + lengths[1] * math.sin(theta2)
		dirForearm = Vector(xEnd, yEnd, zEnd, xElbow, yElbow, zElbow, lengths[2])
		print(dirForearm.get_direction())

		# Using ZYZ rotation matrix:
		theta4 = np.arctan2(np.sqrt(1-np.cos(beta)**2), np.cos(beta))
		theta5 = np.arctan2(np.sin(alpha)*np.sin(beta), np.cos(alpha)*np.sin(beta))
		theta6 = np.arctan2(np.sin(beta)*np.sin(gamma), -np.cos(gamma)*np.sin(beta))
		# print(f'\nxSw: {xSw}, ySw: {ySw}, zSw: {zSw}')
		# print(f'\nAngles in radians:\ntheta1: {theta1}\ntheta2: {theta2}\ntheta3: {theta3}\ntheta4: {theta4}\ntheta5: {theta5}\ntheta6: {theta6}')
		# print(f'\nAngles in degrees:\ntheta1: {theta1*180/np.pi}\ntheta2: {theta2*180/np.pi}\ntheta3: {theta3*180/np.pi}\ntheta4: {theta4*180/np.pi}\ntheta5: {theta5*180/np.pi}\ntheta6: {theta6*180/np.pi}')

# x, y, z, a, b, g, (abg in rads)
invPose(300, 300, 500, 0, 0, 0)