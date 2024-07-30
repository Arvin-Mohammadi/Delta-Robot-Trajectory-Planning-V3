

# =================================================================================================
# -- imports --------------------------------------------------------------------------------------
# =================================================================================================

from SimpleMath 		import tand, sind, cosd
from PathPlannerPTP 	import PathPlannerPTP
from PathPlannerMLTP 	import PathPlannerMLTP

import os 
import math 
import itertools
import numpy as np 
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D


# =================================================================================================
# -- adept cycle -----------------------------------------------------------
# =================================================================================================

class PathPlanner_AdeptCycle:
	def __init__(self, path):
		[self.x, self.y, self.z] = np.transpose(path)

	def cubic_spline(self):
		x_planner 		= PathPlannerMLTP(self.x)
		x_results 		= x_planner.mltp_cubicspline()
		x_interpolated 	= np.array(x_results[1])

		y_planner 		= PathPlannerMLTP(self.y)
		y_results 		= y_planner.mltp_cubicspline()
		y_interpolated 	= np.array(y_results[1])

		z_planner 		= PathPlannerMLTP(self.z)
		z_results 		= z_planner.mltp_cubicspline()
		z_interpolated 	= np.array(z_results[1])

		xyz_array 		= np.array([x_interpolated, y_interpolated, z_interpolated])
		xyz_results 	= [x_results, y_results, z_results]

		return xyz_array, xyz_results



	def point_to_point(self, method_name):
		x_planner_0t1 = PathPlannerPTP(self.x[0],self.x[1])
		x_planner_1t2 = PathPlannerPTP(self.x[1],self.x[2])
		x_planner_2t3 = PathPlannerPTP(self.x[2],self.x[3])
		y_planner_0t1 = PathPlannerPTP(self.y[0],self.y[1])
		y_planner_1t2 = PathPlannerPTP(self.y[1],self.y[2])
		y_planner_2t3 = PathPlannerPTP(self.y[2],self.y[3])
		z_planner_0t1 = PathPlannerPTP(self.z[0],self.z[1])
		z_planner_1t2 = PathPlannerPTP(self.z[1],self.z[2])
		z_planner_2t3 = PathPlannerPTP(self.z[2],self.z[3])

		x_planner_0t1_calculator = getattr(x_planner_0t1, method_name)
		x_planner_1t2_calculator = getattr(x_planner_1t2, method_name) 
		x_planner_2t3_calculator = getattr(x_planner_2t3, method_name)
		y_planner_0t1_calculator = getattr(y_planner_0t1, method_name)
		y_planner_1t2_calculator = getattr(y_planner_1t2, method_name)
		y_planner_2t3_calculator = getattr(y_planner_2t3, method_name)
		z_planner_0t1_calculator = getattr(z_planner_0t1, method_name)
		z_planner_1t2_calculator = getattr(z_planner_1t2, method_name)
		z_planner_2t3_calculator = getattr(z_planner_2t3, method_name)

		x_interpolated 	= 	list(itertools.chain(	x_planner_0t1_calculator()[1], \
													x_planner_1t2_calculator()[1], \
													x_planner_2t3_calculator()[1]))
		y_interpolated 	= 	list(itertools.chain(	y_planner_0t1_calculator()[1], \
													y_planner_1t2_calculator()[1], \
													y_planner_2t3_calculator()[1]))
		z_interpolated 	= 	list(itertools.chain(	z_planner_0t1_calculator()[1], \
													z_planner_1t2_calculator()[1], \
													z_planner_2t3_calculator()[1]))

		x_dot 			=	list(itertools.chain(	x_planner_0t1_calculator()[2], \
													x_planner_1t2_calculator()[2], \
													x_planner_2t3_calculator()[2]))
		y_dot 			= 	list(itertools.chain(	y_planner_0t1_calculator()[2], \
													y_planner_1t2_calculator()[2], \
													y_planner_2t3_calculator()[2]))
		z_dot 			= 	list(itertools.chain(	z_planner_0t1_calculator()[2], \
													z_planner_1t2_calculator()[2], \
													z_planner_2t3_calculator()[2]))

		x_ddot 			= 	list(itertools.chain(	x_planner_0t1_calculator()[3], \
													x_planner_1t2_calculator()[3], \
													x_planner_2t3_calculator()[3]))
		y_ddot 			= 	list(itertools.chain(	y_planner_0t1_calculator()[3], \
													y_planner_1t2_calculator()[3], \
													y_planner_2t3_calculator()[3]))
		z_ddot 			= 	list(itertools.chain(	z_planner_0t1_calculator()[3], \
													z_planner_1t2_calculator()[3], \
													z_planner_2t3_calculator()[3]))

		if x_planner_0t1_calculator()[4] is not None:

			x_dddot 		= 	list(itertools.chain(	x_planner_0t1_calculator()[4], \
														x_planner_1t2_calculator()[4], \
														x_planner_2t3_calculator()[4]))
			y_dddot 		= 	list(itertools.chain(	y_planner_0t1_calculator()[4], \
														y_planner_1t2_calculator()[4], \
														y_planner_2t3_calculator()[4]))
			z_dddot 		= 	list(itertools.chain(	z_planner_0t1_calculator()[4], \
														z_planner_1t2_calculator()[4], \
														z_planner_2t3_calculator()[4]))
		else: 
			x_dddot = None 
			y_dddot = None 
			z_dddot = None 


		xyz_array 			= np.array([x_interpolated, y_interpolated, z_interpolated])

		t 					= np.linspace(0, 1, len(x_interpolated))
		x_results 			= (t, x_interpolated, x_dot, x_ddot, x_dddot)
		y_results 			= (t, y_interpolated, y_dot, y_ddot, y_dddot)
		z_results 			= (t, z_interpolated, z_dot, z_ddot, z_dddot)
		xyz_results			= [x_results, y_results, z_results]


		return xyz_array, xyz_results

	def plot3d(self, xyz_array, method_name):

	    # Plotting the 3D plot
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')	
		
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

	def plot(self, results, method_name, _format='.png', _file_path='./results - adept cycle/'):
		if not os.path.exists(_file_path):
			os.makedirs(_file_path)
		
		(t, x, x_dot, x_ddot, x_dddot) = results[0]
		(t, y, y_dot, y_ddot, y_dddot) = results[1]
		(t, z, z_dot, z_ddot, z_dddot) = results[2]

		fig = plt.figure()
		fig.set_figheight(15)
		fig.set_figwidth(10)

		# plot theta
		plt.subplot(411)
		plt.plot(t, x, label=r'$x$', linewidth=4)
		plt.plot(t, y, label=r'$y$', linewidth=3)
		plt.plot(t, z, label=r'$z$', linewidth=2)
		plt.legend()
		plt.title(r'value-time', fontsize=20)
		plt.xlabel("time", fontsize=15)
		plt.ylabel("value", fontsize=15)

		# plot theta dot 
		plt.subplot(412)
		plt.plot(t, x_dot, label=r'$\dot{x}$', linewidth=4)
		plt.plot(t, y_dot, label=r'$\dot{y}$', linewidth=3)
		plt.plot(t, z_dot, label=r'$\dot{z}$', linewidth=2)
		plt.legend()
		plt.title(r'first differential value-time', fontsize=20)
		plt.xlabel("time", fontsize=15)
		plt.ylabel("first differential value", fontsize=15)

		# plot theta double dot 
		plt.subplot(413)
		plt.plot(t, x_ddot, label=r'$\ddot{x}$', linewidth=4)
		plt.plot(t, y_ddot, label=r'$\ddot{y}$', linewidth=3)
		plt.plot(t, z_ddot, label=r'$\ddot{z}$', linewidth=2)
		plt.legend()
		plt.title(r'second differential value-time', fontsize=20)
		plt.xlabel("time", fontsize=15)
		plt.ylabel("second differential value", fontsize=15)

		print(x_dddot)
		if x_dddot is not None:
			# plot theta triple dot 
			plt.subplot(414)
			plt.plot(t, x_dddot, label=r'$\dddot{x}$', linewidth=4)
			plt.plot(t, y_dddot, label=r'$\dddot{y}$', linewidth=3)
			plt.plot(t, z_dddot, label=r'$\dddot{z}$', linewidth=2)
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

	# Set the path [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]]
	PATH = [[0, 0, 0], [0.15, 0.15, 0.5], [0.85, 0.85, 0.5], [1, 1, 0]]

	# Path planner class init
	path_planner = PathPlanner_AdeptCycle(PATH)
	
	# getting the results for the cubic spline method
	(xyz_array, xyz_results) = path_planner.cubic_spline()

	# path planner plot 3D 
	# path_planner.plot3d(xyz_array, 'Cubic Spline')
	path_planner.plot(xyz_results, 'Cubic Spline')

# -------------------------------------------------------------------------------------------------

	# Set the path [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]]
	PATH = [[0, 0, 0], [0, 0, 0.5], [1, 1, 0.5], [1, 1, 0]]

	# Path planner class init
	path_planner = PathPlanner_AdeptCycle(PATH)
	
	# getting the results for the cubic spline method
	(xyz_array, xyz_results) = path_planner.point_to_point('ptp_polynomial5th')

	# path planner plot 3D 
	# path_planner.plot3d(xyz_array, 'Point-to-Point 5th order polynomial')
	path_planner.plot(xyz_results, 'Point-to-Point 5th order polynomial')


# -------------------------------------------------------------------------------------------------

	# Set the path [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]]
	PATH = [[0, 0, 0], [0, 0, 0.5], [1, 1, 0.5], [1, 1, 0]]

	# Path planner class init
	path_planner = PathPlanner_AdeptCycle(PATH)
	
	# getting the results for the cubic spline method
	(xyz_array, xyz_results) = path_planner.point_to_point('ptp_polynomial7th')

	# path planner plot 3D 
	# path_planner.plot3d(xyz_array, 'Point-to-Point 7th order polynomial')
	path_planner.plot(xyz_results, 'Point-to-Point 7th order polynomial')


# -------------------------------------------------------------------------------------------------

	# Set the path [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]]
	PATH = [[0, 0, 0], [0, 0, 0.5], [1, 1, 0.5], [1, 1, 0]]

	# Path planner class init
	path_planner = PathPlanner_AdeptCycle(PATH)
	
	# getting the results for the cubic spline method
	(xyz_array, xyz_results) = path_planner.point_to_point('ptp_polynomial9th')

	# path planner plot 3D 
	# path_planner.plot3d(xyz_array, 'Point-to-Point 9th order polynomial')
	path_planner.plot(xyz_results, 'Point-to-Point 9th order polynomial')


# -------------------------------------------------------------------------------------------------

	# Set the path [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]]
	PATH = [[0, 0, 0], [0, 0, 0.5], [1, 1, 0.5], [1, 1, 0]]

	# Path planner class init
	path_planner = PathPlanner_AdeptCycle(PATH)
	
	# getting the results for the cubic spline method
	(xyz_array, xyz_results) = path_planner.point_to_point('ptp_bangbang')

	# path planner plot 3D 
	# path_planner.plot3d(xyz_array, 'Point-to-Point Bang Bang')
	path_planner.plot(xyz_results, 'Point-to-Point Bang Bang')


# -------------------------------------------------------------------------------------------------

	# Set the path [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]]
	PATH = [[0, 0, 0], [0, 0, 0.5], [1, 1, 0.5], [1, 1, 0]]

	# Path planner class init
	path_planner = PathPlanner_AdeptCycle(PATH)
	
	# getting the results for the cubic spline method
	(xyz_array, xyz_results) = path_planner.point_to_point('ptp_trapezoidal')

	# path planner plot 3D 
	# path_planner.plot3d(xyz_array, 'Point-to-Point Trapezoidal')
	path_planner.plot(xyz_results, 'Point-to-Point Trapezoidal')


# -------------------------------------------------------------------------------------------------

	# Set the path [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]]
	PATH = [[0, 0, 0], [0, 0, 0.5], [1, 1, 0.5], [1, 1, 0]]

	# Path planner class init
	path_planner = PathPlanner_AdeptCycle(PATH)
	
	# getting the results for the cubic spline method
	(xyz_array, xyz_results) = path_planner.point_to_point('ptp_scurve')

	# path planner plot 3D 
	# path_planner.plot3d(xyz_array, 'Point-to-Point S-Curve')
	path_planner.plot(xyz_results, 'Point-to-Point S-Curve')
