import numpy as np
  
###
# IK helper functions
###

def MatrixScale(A, k):
	for i,v in enumerate(A):
		A[i] = v * k
	return A

def tran2pos(Ttp):
	Xtp = np.zeros(6)
	Xtp[0] = Ttp[0][3]
	Xtp[1] = Ttp[1][3]
	Xtp[2] = Ttp[2][3]
	Xtp[4] = np.arctan2(np.sqrt(Ttp[2][0]*Ttp[2][0] + Ttp[2][1]*Ttp[2][1]),Ttp[2][2])
	Xtp[3] = np.arctan2(Ttp[1][2]/np.sin(Xtp[4]),Ttp[0][2]/np.sin(Xtp[4]))
	Xtp[5] = np.arctan2(Ttp[2][1]/np.sin(Xtp[4]),-Ttp[2][0]/np.sin(Xtp[4]))
	return Xtp

def pos2tran(Xpt):
	# pos to homogeneous transformation matrix
 
	# first row
	tpt0 = np.cos(Xpt[3])*np.cos(Xpt[4])*np.cos(Xpt[5])-np.sin(Xpt[3])*np.sin(Xpt[5])
	tpt1 = -np.cos(Xpt[3])*np.cos(Xpt[4])*np.sin(Xpt[5])-np.sin(Xpt[3])*np.cos(Xpt[5])
	tpt2 = np.cos(Xpt[3])*np.sin(Xpt[4])
	tpt3 = Xpt[0]
	# second row
	tpt4 = np.sin(Xpt[3])*np.cos(Xpt[4])*np.cos(Xpt[5])+np.cos(Xpt[3])*np.sin(Xpt[5])
	tpt5 = -np.sin(Xpt[3])*np.cos(Xpt[4])*np.sin(Xpt[5])+np.cos(Xpt[3])*np.cos(Xpt[5])
	tpt6 = np.sin(Xpt[3])*np.sin(Xpt[4])
	tpt7 = Xpt[1]
	# third row
	tpt8 = -np.sin(Xpt[4])*np.cos(Xpt[5])
	tpt9 = np.sin(Xpt[4])*np.sin(Xpt[5])
	tpt10 = np.cos(Xpt[4])
	tpt11 = Xpt[2]
	# fourth row
	tpt12 = 0.0
	tpt13 = 0.0
	tpt14 = 0.0
	tpt15 = 1.0

	# Build matrix
	Tpt = np.array([[tpt0, tpt1, tpt2, tpt3],
                 	[tpt4, tpt5, tpt6, tpt7],
                  	[tpt8, tpt9, tpt10, tpt11],
                   	[tpt12, tpt13, tpt14, tpt15]])
	return Tpt

def invtran(Titi):
	# finding the inverse of the homogeneous transformation matrix
 
	# first row
	titf0 = Titi[0][0]
	titf1 = Titi[1][0]
	titf2 = Titi[2][0]
	titf3 = -Titi[0][0]*Titi[0][3]-Titi[1][0]*Titi[1][3]-Titi[2][0]*Titi[2][3]
	# second row
	titf4 = Titi[0][1]
	titf5 = Titi[1][1]
	titf6 = Titi[2][1]
	titf7 = -Titi[0][1]*Titi[0][3]-Titi[1][1]*Titi[1][3]-Titi[2][1]*Titi[2][3]
	# third row
	titf8 = Titi[0][2]
	titf9 = Titi[1][2]
	titf10 = Titi[2][2]
	titf11 = -Titi[0][2]*Titi[0][3]-Titi[1][2]*Titi[1][3]-Titi[2][2]*Titi[2][3]
	# forth row
	titf12 = 0.0
	titf13 = 0.0
	titf14 = 0.0
	titf15 = 1.0
 
	# Build matrix
	Titf = np.array([[titf0, titf1, titf2, titf3],
                 	[titf4, titf5, titf6, titf7],
                  	[titf8, titf9, titf10, titf11],
                   	[titf12, titf13, titf14, titf15]])
 
	return Titf

def DH1line(thetadh, alfadh, rdh, ddh):
	# creats Denavit-Hartenberg homogeneous transformation matrix
 
	# first row
	tdh0 = np.cos(thetadh)
	tdh1 = -np.sin(thetadh)*np.cos(alfadh)
	tdh2 = np.sin(thetadh)*np.sin(alfadh)
	tdh3 = rdh*np.cos(thetadh)
	# second row
	tdh4 = np.sin(thetadh)
	tdh5 = np.cos(thetadh)*np.cos(alfadh)
	tdh6 = -np.cos(thetadh)*np.sin(alfadh)
	tdh7 = rdh*np.sin(thetadh)
	# third row
	tdh8 = 0.0
	tdh9 = np.sin(alfadh)
	tdh10 = np.cos(alfadh)
	tdh11 = ddh
	# forth row
	tdh12 = 0.0
	tdh13 = 0.0
	tdh14 = 0.0
	tdh15 = 1.0
 
	# Build matrix
	Tdh = np.array([[tdh0, tdh1, tdh2, tdh3],
                 	[tdh4, tdh5, tdh6, tdh7],
                  	[tdh8, tdh9, tdh10, tdh11],
                   	[tdh12, tdh13, tdh14, tdh15]])
 
	return Tdh