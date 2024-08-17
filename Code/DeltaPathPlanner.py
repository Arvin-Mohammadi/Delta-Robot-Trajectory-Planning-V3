
# =================================================================================================
# -- imports --------------------------------------------------------------------------------------
# =================================================================================================

from SimpleMath import tand, sind, cosd
from DeltaKinematics import DeltaKinematics
from PathPlannerMLTP import PathPlannerMLTP

import os 
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


	def trajectory_gen(self, method_name):
	
		t 				= [] 
		xyz 			= []
		xyz_dot			= []
		xyz_ddot		= []
		xyz_dddot		= []

		for theta in [self.x, self.y, self.z]: 
			planner 	= PathPlannerMLTP(theta)

			results = None 
			if 		method_name == 'mltp_cubicspline':
				results 	= planner.mltp_cubicspline()
			elif 	method_name == 'mltp_polynomial7th_4point':
				results = planner.mltp_polynomial7th_4point()
			elif 	method_name == 'mltp_polynomial9th_4point':
				results = planner.mltp_polynomial9th_4point()
			elif 	method_name == 'mltp_polynomial11th_4point':
				results = planner.mltp_polynomial11th_4point()
			elif 	method_name == 'ptp_polynomial5th':
				results = planner.mltp_ptpmethods('ptp_polynomial5th')
			elif 	method_name == 'ptp_polynomial7th':
				results = planner.mltp_ptpmethods('ptp_polynomial7th')
			elif 	method_name == 'ptp_polynomial9th':
				results = planner.mltp_ptpmethods('ptp_polynomial9th')
			elif 	method_name == 'ptp_bangbang':
				results = planner.mltp_ptpmethods('ptp_bangbang')
			elif 	method_name == 'ptp_trapezoidal':
				results = planner.mltp_ptpmethods('ptp_trapezoidal')
			elif 	method_name == 'ptp_scurve':
				results = planner.mltp_ptpmethods('ptp_scurve')
			
			(t, theta, theta_dot, theta_ddot, theta_dddot) = results 
			xyz.append(theta)
			xyz_dot.append(theta_dot)
			xyz_ddot.append(theta_ddot)
			xyz_dddot.append(theta_dddot)

		return (t, xyz, xyz_dot, xyz_ddot, xyz_dddot)



	def plot3d(self, xyz_array, method_name):

		# Create a 3D plot directly
		ax = plt.axes(projection='3d')
		
		# Extract the interpolated x, y, z values from the xyz_array
		x_interpolated = xyz_array[0]
		y_interpolated = xyz_array[1]
		z_interpolated = xyz_array[2]	
		
		# Plot the cubic spline path
		ax.plot(x_interpolated, y_interpolated, z_interpolated, label=method_name, color='b')	
		
		# Plot the original path points
		original_x, original_y, original_z = zip(*PATH)
		ax.scatter(original_x, original_y, original_z, color='r', marker='o', label='Original Path Points')	
		
		# Set labels
		ax.set_xlabel('X axis')
		ax.set_ylabel('Y axis')
		ax.set_zlabel('Z axis')	
		
		# Set title
		ax.set_title('3D Trajectory - Method: ' + method_name)
		
		# Show legend
		ax.legend()	

		# Show the plot
		plt.show()


	def plot(self, results, method_name, _format='.pdf', _file_path='./results - adept cycle/'):
		if not os.path.exists(_file_path):
			os.makedirs(_file_path)
		
		(t, xyz, xyz_dot, xyz_ddot, xyz_dddot) = results

		x = xyz[0]
		y = xyz[1]
		z = xyz[2]

		x_dot = xyz_dot[0] 
		y_dot = xyz_dot[1]
		z_dot = xyz_dot[2]

		x_ddot = xyz_ddot[0] 
		y_ddot = xyz_ddot[1]
		z_ddot = xyz_ddot[2]

		x_dddot = xyz_dddot[0] 
		y_dddot = xyz_dddot[1]
		z_dddot = xyz_dddot[2]

		fig = plt.figure()
		fig.set_figheight(15)
		fig.set_figwidth(10)

		# plot theta
		plt.subplot(411)
		plt.plot(t, x, label=r'$x$', linewidth=5)
		plt.plot(t, y, label=r'$y$', linewidth=3)
		plt.plot(t, z, label=r'$z$', linewidth=1)
		plt.legend()
		plt.title(r'value-time', fontsize=20)
		plt.xlabel("time", fontsize=15)
		plt.ylabel("value", fontsize=15)

		# plot theta dot 
		plt.subplot(412)
		plt.plot(t, x_dot, label=r'$\dot{x}$', linewidth=5)
		plt.plot(t, y_dot, label=r'$\dot{y}$', linewidth=3)
		plt.plot(t, z_dot, label=r'$\dot{z}$', linewidth=1)
		plt.legend()
		plt.title(r'first differential value-time', fontsize=20)
		plt.xlabel("time", fontsize=15)
		plt.ylabel("first differential value", fontsize=15)

		# plot theta double dot 
		plt.subplot(413)
		plt.plot(t, x_ddot, label=r'$\ddot{x}$', linewidth=5)
		plt.plot(t, y_ddot, label=r'$\ddot{y}$', linewidth=3)
		plt.plot(t, z_ddot, label=r'$\ddot{z}$', linewidth=1)
		plt.legend()
		plt.title(r'second differential value-time', fontsize=20)
		plt.xlabel("time", fontsize=15)
		plt.ylabel("second differential value", fontsize=15)

		if x_dddot is not None:
			# plot theta triple dot 
			plt.subplot(414)
			plt.plot(t, x_dddot, label=r'$\dddot{x}$', linewidth=5)
			plt.plot(t, y_dddot, label=r'$\dddot{y}$', linewidth=3)
			plt.plot(t, z_dddot, label=r'$\dddot{z}$', linewidth=1)
			plt.legend()
			plt.title(r'third differential value-time', fontsize=20)
			plt.xlabel("time", fontsize=15)
			plt.ylabel("third differential value", fontsize=15)

		plt.tight_layout()
		plt.savefig(_file_path + method_name + _format)
		plt.clf()

		
# =================================================================================================
# -- main -----------------------------------------------------------------------------------------
# =================================================================================================

if __name__ == "__main__":

	# Set the path 
	PATH = [[0, 0, 0], [0, 0, 0.5], [1, 1, 0.5], [1, 1, 0]]

	# init the robot 
	robot = DeltaKinematics(0.2, 0.46, 0.1, 0.074)
	# initialize the path planner class 
	path_planner = DeltaPathPlanner(robot,PATH)

	# calculate the trajectory based on cubic spline 
	results = path_planner.trajectory_gen("mltp_cubicspline")
	path_planner.plot(results, "adept mltp_cubicspline")
	path_planner.plot3d(results[1], "adept cycle cubic spline")

	# multi-point high order polynomials 
	results = path_planner.trajectory_gen('mltp_polynomial7th_4point')
	path_planner.plot(results, "mltp_polynomial7th_4point")
	path_planner.plot3d(results[1], "adept cycle 4 point polynoimal 7th order")

	results = path_planner.trajectory_gen('mltp_polynomial9th_4point')
	path_planner.plot(results, "mltp_polynomial9th_4point")
	path_planner.plot3d(results[1], "adept cycle 4 point polynoimal 9th order")

	results = path_planner.trajectory_gen('mltp_polynomial11th_4point')
	path_planner.plot(results, "mltp_polynomial11th_4point")
	path_planner.plot3d(results[1], "adept cycle 4 point polynoimal 11th order")

	# point to point methods
	results = path_planner.trajectory_gen('ptp_polynomial5th')
	path_planner.plot(results, 'ptp_polynomial5th')
	path_planner.plot3d(results[1], 'point to point 5th order polynomial')

	results = path_planner.trajectory_gen('ptp_polynomial7th')
	path_planner.plot(results, 'ptp_polynomial7th')
	path_planner.plot3d(results[1], 'point to point 7th order polynomial')

	results = path_planner.trajectory_gen('ptp_polynomial9th')
	path_planner.plot(results, 'ptp_polynomial9th')
	path_planner.plot3d(results[1], 'point to point 9th order polynomial')

	results = path_planner.trajectory_gen('ptp_bangbang')
	path_planner.plot(results, 'ptp_bangbang')
	path_planner.plot3d(results[1], 'point to point parabolic method')

	results = path_planner.trajectory_gen('ptp_trapezoidal')
	path_planner.plot(results, 'ptp_trapezoidal')
	path_planner.plot3d(results[1], 'point to point trapezoidal')

	results = path_planner.trajectory_gen('ptp_scurve')
	path_planner.plot(results, 'ptp_scurve')
	path_planner.plot3d(results[1], 'point to point S-curve')

