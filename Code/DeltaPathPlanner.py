
# =================================================================================================
# -- imports --------------------------------------------------------------------------------------
# =================================================================================================

from SimpleMath import tand, sind, cosd
from DeltaKinematics import DeltaKinematics

import math 
import numpy as np 
import matplotlib.pyplot as plt 


# =================================================================================================
# -- the path planner for the delta robot 3D ------------------------------------------------------
# =================================================================================================

class DeltaPathPlanner:

	def __init__(self, robot, path):
		self.robot = robot 
		[self.x, self.y, self.z] = np.transpose(path)


	def cubic_spline(self):
	
		t 				= [] 
		xyz 			= []
		xyz_dot			= []
		xyz_ddot		= []
		xyz_dddot		= []

		for theta in [self.x, self.y, self.z]: 
			planner 	= PathPlannerMLTP(theta)
			results 	= planner.mltp_cubicspline()
			(t, theta, theta_dot, theta_ddot, theta_dddot) = results 
			xyz.append(theta)
			xyz_dot.append(theta_dot)
			xyz_ddot.append(theta_ddot)
			xyz_dddot.append(theta_dddot)

		return (t, xyz, xyz_dot, xyz_ddot, xyz_dddot)


		
# =================================================================================================
# -- main -----------------------------------------------------------------------------------------
# =================================================================================================

if __name__ == "__main__":

	# Set the path 
	PATH = [0, -0.2, 0.3, 0.8, -0.1, 1]

	# initialize the path planner class 
	path_planner = PathPlannerMLTP(PATH)

	# calculate the trajectory based on cubic spline 
	results = path_planner.mltp_cubicspline()
	path_planner.plot(results, "cubic spline")


