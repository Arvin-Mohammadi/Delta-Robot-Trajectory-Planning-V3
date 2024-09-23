
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
from numpy import floor 


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
				results = planner.mltp_cubicspline()
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
			try: 
				xyz_dddot.append(theta_dddot)
			except: 
				pass 

			if theta_dddot is None:
				xyz_dddot = None 

		return (t, xyz, xyz_dot, xyz_ddot, xyz_dddot)

	def plot3d(self, xyz_array, method_name, ax=None):

		# Create a 3D plot directly
		if ax is None: ax = plt.axes(projection='3d')
		
		# Extract the interpolated x, y, z values from the xyz_array
		x_interpolated = xyz_array[0]
		y_interpolated = xyz_array[1]
		z_interpolated = xyz_array[2]	
		
		# Plot the cubic spline path
		linestyle = 'b'
		ax.plot(x_interpolated, y_interpolated, z_interpolated, linestyle, label="Theoretical", linewidth=2)	
		
		# Plot the original path points
		original_x, original_y, original_z = zip(*PATH)
		ax.scatter(original_x, original_y, original_z, color='r', marker='o')	
		
		# Hide axis numbers
		plt.gca().set_xticks([])
		plt.gca().set_yticks([])
		plt.gca().set_zticks([])
		
		# Set title
		# ax.set_title('3D Trajectory: ' + method_name)
		
		plt.tight_layout()
		plt.show()


	def plot3d_practical_results(self, xyz_array, method_name):
	
		AX = plt.axes(projection='3d')

		self.plot3d(xyz_array, method_name, ax=AX)

		for i in range(10):

			if i == 0:
				[X, Y, Z] = np.load("results - practical test/" + "ee_history_" + method_name + str(i) + ".npy").transpose()
				AX.plot(X, Y, Z, label="Experimental - CW", color='b', linewidth=1)
				continue 			
			if i == 1:  
				[X, Y, Z] = np.load("results - practical test/" + "ee_history_" + method_name + str(i) + ".npy").transpose()
				AX.plot(X, Y, Z, label="Experimental - CCW", color='c', linewidth=1)
				continue 

			if i % 2 == 0:
				[X, Y, Z] = np.load("results - practical test/" + "ee_history_" + method_name + str(i) + ".npy").transpose()
				AX.plot(X, Y, Z, color='b', linewidth=1)
			else: 
				[X, Y, Z] = np.load("results - practical test/" + "ee_history_" + method_name + str(i) + ".npy").transpose()
				AX.plot(X, Y, Z, color='c', linewidth=1)

		plt.legend()
		plt.tight_layout()
		plt.show()

		
	def plot_practical_results(self, results, results_reverse, method_name, _format='.pdf', _file_path='./results - practical test plot/'):

		(_, xyz, _, _, _) = results
		(_, xyz_rev, _, _, _) = results_reverse

		x = xyz[0]
		y = xyz[1]
		z = xyz[2]

		x_rev = xyz_rev[0]
		y_rev = xyz_rev[1]
		z_rev = xyz_rev[2]

		fig = plt.figure()
		fig.set_figheight(5)
		fig.set_figwidth(10)

		# Set global font to Times New Roman
		plt.rc('font', family='Times New Roman')


		for i in range(5):

			# loading cycle and reverse cycle raw data 
			[X, Y, Z] 			= np.load("results - practical test/" + "ee_history_" + method_name + str(i*2) + ".npy").transpose()
			[Xrev, Yrev, Zrev] 	= np.load("results - practical test/" + "ee_history_" + method_name + str(i*2+1) + ".npy").transpose()
			T 					= np.load("results - practical test/" + "time_history_" + method_name + str(i*2) + ".npy")
			Trev 				= np.load("results - practical test/" + "time_history_" + method_name + str(i*2+1) + ".npy")

			# concat the cycle and reverse cycle 
			Xfull = np.concatenate((X, Xrev), axis=None)
			Yfull = np.concatenate((Y, Yrev), axis=None)
			Zfull = np.concatenate((Z, Zrev), axis=None)
			
			# building index full timeline  
			temp 		= Trev + 1 
			temp		= np.concatenate((T, temp), axis=None)
			Tfull 		= temp * 2

			# building index vector
			T2index		= floor((len(x)-1)*T)
			Trev2index  = floor((len(x)-1)*Trev)

			# computing the errors 
			x_error = X-[x[int(i)] for i in T2index]
			y_error = Y-[y[int(i)] for i in T2index]
			z_error = Z-[z[int(i)] for i in T2index]

			xrev_error = Xrev-[x_rev[int(i)] for i in Trev2index]
			yrev_error = Yrev-[y_rev[int(i)] for i in Trev2index]
			zrev_error = Zrev-[z_rev[int(i)] for i in Trev2index]

			xfull_error = np.concatenate((x_error, xrev_error), axis=None) 
			yfull_error = np.concatenate((y_error, yrev_error), axis=None) 
			zfull_error = np.concatenate((z_error, zrev_error), axis=None) 

			# plotting the errors 
			if i == 0:
				plt.plot(Tfull, xfull_error, 	'b', label=r'$x$', linewidth=1)
				plt.plot(Tfull, yfull_error, 	'r', label=r'$y$', linewidth=1)
				plt.plot(Tfull, zfull_error, 	'm', label=r'$z$', linewidth=1)
			else:
				plt.plot(Tfull, xfull_error, 	'b', linewidth=1)
				plt.plot(Tfull, yfull_error, 	'r', linewidth=1)
				plt.plot(Tfull, zfull_error, 	'm', linewidth=1)


		plt.xlabel(r"$t (s)$", fontsize=28)
		plt.ylabel(r"$error (cm)$", fontsize=28)
		plt.tick_params(axis='both', which='major', labelsize=22)  # Increase axis numbers font size
		plt.legend(fontsize=22)
		plt.tight_layout()
		plt.savefig(_file_path + method_name + _format)


	def plot_practical_results_distance(self, results, results_reverse, method_name, _format='.pdf', _file_path='./results - practical test plot distance error/'):
		if not os.path.exists(_file_path):
			os.makedirs(_file_path)

		(_, xyz, _, _, _) = results
		(_, xyz_rev, _, _, _) = results_reverse

		x = xyz[0]
		y = xyz[1]
		z = xyz[2]

		x_rev = xyz_rev[0]
		y_rev = xyz_rev[1]
		z_rev = xyz_rev[2]

		fig = plt.figure()
		fig.set_figheight(5)
		fig.set_figwidth(10)

		# Set global font to Times New Roman
		plt.rc('font', family='Times New Roman')

		max_error 		= 0
		mean_error 		= 0 
		variance_error 	= 0 
		final_error 	= 0

		for i in range(1):

			# loading cycle and reverse cycle raw data 
			[X, Y, Z] 			= np.load("results - practical test/time 5/" + "ee_history_" + method_name + str(i*2) + ".npy").transpose()
			[Xrev, Yrev, Zrev] 	= np.load("results - practical test/time 5/" + "ee_history_" + method_name + str(i*2+1) + ".npy").transpose()
			T 					= np.load("results - practical test/time 5/" + "time_history_" + method_name + str(i*2) + ".npy")
			Trev 				= np.load("results - practical test/time 5/" + "time_history_" + method_name + str(i*2+1) + ".npy")

			# concat the cycle and reverse cycle 
			Xfull = np.concatenate((X, Xrev), axis=None)
			Yfull = np.concatenate((Y, Yrev), axis=None)
			Zfull = np.concatenate((Z, Zrev), axis=None)
			
			# building index full timeline  
			temp 		= Trev + 1 
			temp		= np.concatenate((T, temp), axis=None)
			Tfull 		= temp * 2

			# building index vector
			T2index		= floor((len(x)-1)*T)
			Trev2index  = floor((len(x)-1)*Trev)

			# computing the errors 
			x_error = X-[x[int(i)] for i in T2index]
			y_error = Y-[y[int(i)] for i in T2index]
			z_error = Z-[z[int(i)] for i in T2index]

			xrev_error = Xrev-[x_rev[int(i)] for i in Trev2index]
			yrev_error = Yrev-[y_rev[int(i)] for i in Trev2index]
			zrev_error = Zrev-[z_rev[int(i)] for i in Trev2index]

			xfull_error = np.concatenate((x_error, xrev_error), axis=None) 
			yfull_error = np.concatenate((y_error, yrev_error), axis=None) 
			zfull_error = np.concatenate((z_error, zrev_error), axis=None)

			distance_error = [(xfull_error[i]**2 + yfull_error[i]**2 + zfull_error[i]**2)**0.5 for i in range(len(xfull_error))]
			distance_error_array = np.array(distance_error)

			# compute the max error 
			max_error += np.max(distance_error_array)

			# computer the mean error 
			mean_error += np.mean(distance_error_array)

			# compute the variance error 
			variance_error += np.var(distance_error_array)

			# compute the final error 
			final_error += distance_error[-1]

			# plotting the errors 
			plt.plot(Tfull, distance_error, 	'b', linewidth=1)


		plt.xlabel(r"$t (s)$", fontsize=28)
		plt.ylabel(r"$error (cm)$", fontsize=28)
		plt.tick_params(axis='both', which='major', labelsize=22)  # Increase axis numbers font size
		plt.tight_layout()
		plt.savefig(_file_path + method_name + _format)


		max_error = max_error/5
		mean_error = mean_error/5
		variance_error = variance_error/5
		final_error = final_error/5

		print("Method: ", 			method_name)
		print("Max Error: ", 		max_error)
		print("Mean Error: ", 		mean_error)
		print("Variance Error: ", 	variance_error)
		print("Final Error: ", 		final_error)
		print(" ")


	def plot(self, results, method_name, _format='.pdf', _file_path='./results - adept cycle/', fig=None):
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
		
		if fig is None:
			fig = plt.figure()
			fig.set_figheight(15)
			fig.set_figwidth(10)


		# Set global font to Times New Roman
		plt.rc('font', family='Times New Roman')

		if xyz_dddot is not None: 

			x_dddot = xyz_dddot[0] 
			y_dddot = xyz_dddot[1]
			z_dddot = xyz_dddot[2]

			# plot theta
			plt.subplot(411)
			plt.plot(t, x, label=r'$x$', linewidth=8)
			plt.plot(t, y, label=r'$y$', linewidth=6)
			plt.plot(t, z, label=r'$z$', linewidth=4)
			plt.legend(fontsize=22)
			plt.xlabel(r"$t$", fontsize=28)
			plt.ylabel(r"$p$", fontsize=28)
			plt.tick_params(axis='both', which='major', labelsize=22)  # Increase axis numbers font size


			# plot theta dot 
			plt.subplot(412)
			plt.plot(t, x_dot, label=r'$\dot{x}$', linewidth=8)
			plt.plot(t, y_dot, label=r'$\dot{y}$', linewidth=6)
			plt.plot(t, z_dot, label=r'$\dot{z}$', linewidth=4)
			plt.legend(fontsize=22)
			plt.xlabel(r"$t$", fontsize=28)
			plt.ylabel(r"$\dot{p}$", fontsize=28)
			plt.tick_params(axis='both', which='major', labelsize=22)  # Increase axis numbers font size


			# plot theta double dot 
			plt.subplot(413)
			plt.plot(t, x_ddot, label=r'$\ddot{x}$', linewidth=8)
			plt.plot(t, y_ddot, label=r'$\ddot{y}$', linewidth=6)
			plt.plot(t, z_ddot, label=r'$\ddot{z}$', linewidth=4)
			plt.legend(fontsize=22)
			plt.xlabel(r"$t$", fontsize=28)
			plt.ylabel(r"$\ddot{p}$", fontsize=28)
			plt.tick_params(axis='both', which='major', labelsize=22)  # Increase axis numbers font size


			# plot theta triple dot 
			plt.subplot(414)
			plt.plot(t, x_dddot, label=r'$\dddot{x}$', linewidth=8)
			plt.plot(t, y_dddot, label=r'$\dddot{y}$', linewidth=6)
			plt.plot(t, z_dddot, label=r'$\dddot{z}$', linewidth=4)
			plt.legend(fontsize=22)
			plt.xlabel(r"$t$", fontsize=28)
			plt.ylabel(r"$\dddot{p}$", fontsize=28)
			plt.tick_params(axis='both', which='major', labelsize=22)  # Increase axis numbers font size

		else: 
			# plot theta
			plt.subplot(311)
			plt.plot(t, x, label=r'$x$', linewidth=8)
			plt.plot(t, y, label=r'$y$', linewidth=6)
			plt.plot(t, z, label=r'$z$', linewidth=4)
			plt.legend(fontsize=22)
			plt.xlabel(r"$t$", fontsize=28)
			plt.ylabel(r"$p$", fontsize=28)
			plt.tick_params(axis='both', which='major', labelsize=22)  # Increase axis numbers font size


			# plot theta dot 
			plt.subplot(312)
			plt.plot(t, x_dot, label=r'$\dot{x}$', linewidth=8)
			plt.plot(t, y_dot, label=r'$\dot{y}$', linewidth=6)
			plt.plot(t, z_dot, label=r'$\dot{z}$', linewidth=4)
			plt.legend(fontsize=22)
			plt.xlabel(r"$t$", fontsize=28)
			plt.ylabel(r"$\dot{p}$", fontsize=28)
			plt.tick_params(axis='both', which='major', labelsize=22)  # Increase axis numbers font size


			# plot theta double dot 
			plt.subplot(313)
			plt.plot(t, x_ddot, label=r'$\ddot{x}$', linewidth=8)
			plt.plot(t, y_ddot, label=r'$\ddot{y}$', linewidth=6)
			plt.plot(t, z_ddot, label=r'$\ddot{z}$', linewidth=4)
			plt.legend(fontsize=22)
			plt.xlabel(r"$t$", fontsize=28)
			plt.ylabel(r"$\ddot{p}$", fontsize=28)
			plt.tick_params(axis='both', which='major', labelsize=22)  # Increase axis numbers font size



# =================================================================================================
# -- main -----------------------------------------------------------------------------------------
# =================================================================================================

if __name__ == "__main__":


	# Set the path 
	PATH = [[-5, 5, -50], [-5, 5, -45], [5, -5, -45], [5, -5, -50]]
	PATH_revERSE = [[5, -5, -50], [5, -5, -45], [-5, 5, -45], [-5, 5, -50]]
	# PATH = [[0, 0, 0], [0, 0, 0.5], [1, 1, 0.5], [1, 1, 0]]

	# initialize the path planner class 
	path_planner = DeltaPathPlanner(None,PATH)
	path_planner_reverse = DeltaPathPlanner(None,PATH_revERSE)

	methods = [	'mltp_cubicspline', 'mltp_polynomial7th_4point', 'mltp_polynomial9th_4point', \
				'mltp_polynomial11th_4point', 'ptp_polynomial5th', 'ptp_polynomial7th', 'ptp_polynomial9th', \
				'ptp_bangbang', 'ptp_trapezoidal','ptp_scurve']


	for method in methods: 
		results 		= path_planner.trajectory_gen(method)
		results_reverse = path_planner_reverse.trajectory_gen(method)
		# path_planner.plot_practical_results(results, results_reverse, method)
		path_planner.plot_practical_results_distance(results, results_reverse, method)
