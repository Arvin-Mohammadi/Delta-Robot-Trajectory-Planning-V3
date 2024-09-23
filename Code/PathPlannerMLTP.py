

# =================================================================================================
# -- imports --------------------------------------------------------------------------------------
# =================================================================================================

from SimpleMath import tand, sind, cosd
from PathPlannerPTP import PathPlannerPTP

import os 
import math 
import numpy as np 
import matplotlib.pyplot as plt 

# =================================================================================================
# -- mutli point path planning class -----------------------------------------------------------
# =================================================================================================

class PathPlannerMLTP: 
	def __init__(self, path, sampling_frequency=100):
		'''
		A Trajectory Generation Class based on a given path
		'''

		# primary values and vectors 
		self.path 				= np.array(path)
		self.n 					= len(self.path) - 1 
		# number of overall trajectory samples 
		self.sampling_frequency = sampling_frequency
		self.sampling_number 	= sampling_frequency*self.n + 1
		# time vector corresponding to path points 
		self.t_path				= np.linspace(0, 1, self.n+1)
		# vector of time intervals for the t_path
		self.T_path 			= self.t_path[1:self.n+1] - self.t_path[0:self.n]


	def mltp_ptpmethods(self, method_name):
		
		t      			= np.linspace(0, 1, self.sampling_number)
		theta      		= np.zeros_like(t)
		theta_dot  		= np.zeros_like(t)
		theta_ddot      = np.zeros_like(t)
		theta_dddot 	= np.zeros_like(t)

		for i in range(len(self.path)-1):
			path_planner = PathPlannerPTP(self.path[i], self.path[i+1], sampling_frequency=self.sampling_frequency)

			# init 
			(_t, _theta, _theta_dot, _theta_ddot, _theta_dddot) = (None, None, None, None, None)

			if method_name == "ptp_polynomial5th":
				(_t, _theta, _theta_dot, _theta_ddot, _theta_dddot) = path_planner.ptp_polynomial5th()
			elif method_name == "ptp_polynomial7th":
				(_t, _theta, _theta_dot, _theta_ddot, _theta_dddot) = path_planner.ptp_polynomial7th()
			elif method_name == "ptp_polynomial9th":
				(_t, _theta, _theta_dot, _theta_ddot, _theta_dddot) = path_planner.ptp_polynomial9th()
			elif method_name == "ptp_bangbang":
				(_t, _theta, _theta_dot, _theta_ddot, _theta_dddot) = path_planner.ptp_bangbang()
			elif method_name == "ptp_trapezoidal":
				(_t, _theta, _theta_dot, _theta_ddot, _theta_dddot) = path_planner.ptp_trapezoidal()
			elif method_name == "ptp_scurve":
				(_t, _theta, _theta_dot, _theta_ddot, _theta_dddot) = path_planner.ptp_scurve()

			theta[i*self.sampling_frequency:(i+1)*self.sampling_frequency+1] 		= _theta
			theta_dot[i*self.sampling_frequency:(i+1)*self.sampling_frequency+1] 	= _theta_dot
			theta_ddot[i*self.sampling_frequency:(i+1)*self.sampling_frequency+1] 	= _theta_ddot
			theta_dddot[i*self.sampling_frequency:(i+1)*self.sampling_frequency+1] 	= _theta_dddot

		if method_name in ["ptp_trapezoidal", "ptp_bangbang"]:
			theta_dddot = None 

		return t, theta, theta_dot, theta_ddot, theta_dddot

	def mltp_polynomial7th_4point(self):
		
		if len(self.path) != 4:
			print("The length of the path is not equal to four")
			return None 

		theta0 = self.path[0]
		theta1 = self.path[1]
		theta2 = self.path[2]
		theta3 = self.path[3]
		t      = np.linspace(0, 1, self.sampling_number)

		a0 = theta0
		a1 = 0
		a2 = 0
		a3 = 182.25 * theta1 - 134.875 * theta0 - 91.125 * theta2 + 43.75 * theta3
		a4 = 548.25 * theta0 - 820.125 * theta1 + 546.75 * theta2 - 274.875 * theta3
		a5 = 1366.875 * theta1 - 856.5 * theta0 - 1093.5 * theta2 + 583.125 * theta3
		a6 = 600.75 * theta0 - 1002.375 * theta1 + 911.25 * theta2 - 509.625 * theta3
		a7 = 273.375 * theta1 - 158.625 * theta0 - 273.375 * theta2 + 158.625 * theta3

		theta 		= a7 * t**7 + a6 * t**6 + a5 * t**5 + a4 * t**4 + a3 * t**3 + a2 * t**2 + a1 * t + a0
		theta_dot 	= 7 * a7 * t**6 + 6 * a6 * t**5 + 5 * a5 * t**4 + 4 * a4 * t**3 + 3 * a3 * t**2 + 2 * a2 * t + a1
		theta_ddot 	= 42 * a7 * t**5 + 30 * a6 * t**4 + 20 * a5 * t**3 + 12 * a4 * t**2 + 6 * a3 * t + 2 * a2
		theta_dddot = 210 * a7 * t**4 + 120 * a6 * t**3 + 60 * a5 * t**2 + 24 * a4 * t + 6 * a3

		return t, theta, theta_dot, theta_ddot, theta_dddot


	def mltp_polynomial9th_4point(self):

		if len(self.path) != 4:
			print("The length of the path is not equal to four")
			return None 

		theta0 = self.path[0]
		theta1 = self.path[1]
		theta2 = self.path[2]
		theta3 = self.path[3]
		t      = np.linspace(0, 1, self.sampling_number)

		a0 = theta0
		a1 = 0
		a2 = 0
		a3 = 0
		a4 = 820.125 * theta1 - 641.9375 * theta0 - 410.0625 * theta2 + 231.875 * theta3
		a5 = 3315.5625 * theta0 - 4510.6875 * theta1 + 2870.4375 * theta2 - 1675.3125 * theta3
		a6 = 9841.5 * theta1 - 6926.875 * theta0 - 7381.125 * theta2 + 4466.5 * theta3
		a7 = 7270.625 * theta0 - 10661.625 * theta1 + 9021.375 * theta2 - 5630.375 * theta3
		a8 = 5740.875 * theta1 - 3822.1875 * theta0 - 5330.8125 * theta2 + 3412.125 * theta3
		a9 = 803.8125 * theta0 - 1230.1875 * theta1 + 1230.1875 * theta2 - 803.8125 * theta3

		theta      	= a9 * t**9 + a8 * t**8 + a7 * t**7 + a6 * t**6 + a5 * t**5 + a4 * t**4 + a3 * t**3 + a2 * t**2 + a1 * t + a0
		theta_dot  	= 9 * a9 * t**8 + 8 * a8 * t**7 + 7 * a7 * t**6 + 6 * a6 * t**5 + 5 * a5 * t**4 + 4 * a4 * t**3 + 3 * a3 * t**2 + 2 * a2 * t + a1
		theta_ddot 	= 72 * a9 * t**7 + 56 * a8 * t**6 + 42 * a7 * t**5 + 30 * a6 * t**4 + 20 * a5 * t**3 + 12 * a4 * t**2 + 6 * a3 * t + 2 * a2
		theta_dddot = 504 * a9 * t**6 + 336 * a8 * t**5 + 210 * a7 * t**4 + 120 * a6 * t**3 + 60 * a5 * t**2 + 24 * a4 * t + 6 * a3

		return t, theta, theta_dot, theta_ddot, theta_dddot


	def mltp_polynomial11th_4point(self):

		if len(self.path) != 4:
			print("The length of the path is not equal to four")
			return None 

		theta0 = self.path[0]
		theta1 = self.path[1]
		theta2 = self.path[2]
		theta3 = self.path[3]
		t      = np.linspace(0, 1, self.sampling_number)

		a0 	= theta0
		a1 	= 0
		a2 	= 0
		a3 	= 0
		a4 	= 0
		a5 	= 3690.5625 * theta1 - 3014.71875 * theta0 - 1845.28125 * theta2 + 1169.4375 * theta3
		a6 	= 18795.75 * theta0 - 23988.65625 * theta1 + 14762.25 * theta2 - 9569.34375 * theta3
		a7 	= 64584.84375 * theta1 - 49087.96875 * theta0 - 46132.03125 * theta2 + 30635.15625 * theta3
		a8 	= 68523.75 * theta0 - 92264.0625 * theta1 + 73811.25 * theta2 - 50070.9375 * theta3
		a9 	= 73811.25 * theta1 - 53835.15625 * theta0 - 64584.84375 * theta2 + 44608.75 * theta3
		a10 = 22549.5 * theta0 - 31369.78125 * theta1 + 29524.5 * theta2 - 20704.21875 * theta3
		a11 = 5535.84375 * theta1 - 3932.15625 * theta0 - 5535.84375 * theta2 + 3932.15625 * theta3

		theta      	= a11 * t**11 + a10 * t**10 + a9 * t**9 + a8 * t**8 + a7 * t**7 + a6 * t**6 + a5 * t**5 + a4 * t**4 + a3 * t**3 + a2 * t**2 + a1 * t + a0
		theta_dot  	= 11 * a11 * t**10 + 10 * a10 * t**9 + 9 * a9 * t**8 + 8 * a8 * t**7 + 7 * a7 * t**6 + 6 * a6 * t**5 + 5 * a5 * t**4 + 4 * a4 * t**3 + 3 * a3 * t**2 + 2 * a2 * t + a1
		theta_ddot 	= 110 * a11 * t**9 + 90 * a10 * t**8 + 72 * a9 * t**7 + 56 * a8 * t**6 + 42 * a7 * t**5 + 30 * a6 * t**4 + 20 * a5 * t**3 + 12 * a4 * t**2 + 6 * a3 * t + 2 * a2
		theta_dddot = 990 * a11 * t**8 + 720 * a10 * t**7 + 504 * a9 * t**6 + 336 * a8 * t**5 + 210 * a7 * t**4 + 120 * a6 * t**3 + 60 * a5 * t**2 + 24 * a4 * t + 6 * a3

		return t, theta, theta_dot, theta_ddot, theta_dddot


	def mltp_cubicspline(self):
		''' 
		Generates a trajectory based on Cubic Spline method for the path  
		'''

		# calculate the velocity vector and the coeff matrix 
		velocity_vector = self._cubicspline_velocity()
		coeff_matrix 	= self._cubicspline_coeff(velocity_vector) 
		# time corresponding to position/velocity trajectory samples
		t 				= np.linspace(0, 1, self.sampling_number)
		# build time corresponding to each of the polynomials based on t - t_k 
		t_interval 		= np.copy(t)

		# fix the time intervals 
		counter = 0
		for i, e in enumerate(t):
			t_interval[i] 	= e - self.t_path[counter]
			next_step 		= (e >= self.t_path[counter+1]) and ((counter+1) < len(self.t_path))
			if next_step:
				counter    += 1

		# initialize theta(t)
		theta 			= np.zeros_like(t)
		theta_dot 		= np.zeros_like(t)
		theta_ddot		= np.zeros_like(t)
		theta_dddot		= np.zeros_like(t)

		# Constructing the theta, theta', theta'', theta''' 
		counter = 0 
		for i, e in enumerate(t): 

			theta[i] 			= 	coeff_matrix[counter][0] + \
									coeff_matrix[counter][1]*t_interval[i] + \
									coeff_matrix[counter][2]*t_interval[i]**2 + \
									coeff_matrix[counter][3]*t_interval[i]**3 	

			theta_dot[i] 		= 	coeff_matrix[counter][1] + \
									2*coeff_matrix[counter][2]*t_interval[i] + \
									3*coeff_matrix[counter][3]*t_interval[i]**2 	

			theta_ddot[i] 		= 	2*coeff_matrix[counter][2] + \
									6*coeff_matrix[counter][3]*t_interval[i] 	

			theta_dddot[i] 		= 	6*coeff_matrix[counter][3]	

			

			if (e >= self.t_path[counter+1]) and ((counter+1) < len(self.t_path)):
				counter += 1


		return (t, theta, theta_dot, theta_ddot, theta_dddot)


	def _cubicspline_velocity(self):
		'''
		Helper function for calculating the velocity profile 
		in cubic spline method
		'''
		v_i 			= 0 
		v_f 			= 0 
		A_prime 		= np.zeros((self.n-1, self.n-1))
		c_prime 		= np.zeros((self.n-1))
		velocity_vector = np.zeros_like(self.path)

		# making the A_prime matrix 
		for i in range(self.n-1):
			A_prime[i, i] 			= 2*(self.T_path[i] + self.T_path[i+1])

			if 		i != 0:
				A_prime[i, i-1] 	= self.T_path[i+1]
			if 		i != self.n-2:
				A_prime[i, i+1] 	= self.T_path[i]

		# making the C_prime matrix
		for i in range(self.n-1):
			c_prime[i] 		= 3/(self.T_path[i]*self.T_path[i+1])*(self.T_path[i]**2*(self.path[i+2] - \
								self.path[i+1]) + self.T_path[i+1]**2*(self.path[i+1] - self.path[i]))
			
			if 		i == 0:
				c_prime[i] -= self.T_path[i+1]*v_i
			elif 	i == self.n-2:
				c_prime[i] -= self.T_path[i]*v_f

		# calculating v vector from A_prime and C_prime martices
		M = np.linalg.inv(A_prime)
		N = c_prime
		v = np.matmul(M, N)

		velocity_vector[0] 		= v_i 
		velocity_vector[-1] 	= v_f
		velocity_vector[1:-1] 	= v

		return velocity_vector

	def _cubicspline_coeff(self, velocity_vector):
		'''
		Helper function for calculating the coefficient matrix 
		in cubic spline method

		in the coefficient matrix we have: 

		dimension 0 = number of coefficients in each polynomial
		dimension 1 = number of polynomials
		'''
		coeff = np.zeros((self.n, 4))

		coeff[:, 0] = self.path[0:self.n]
		coeff[:, 1] = velocity_vector[0:self.n]
		coeff[:, 2] = 1/self.T_path*( 3*(self.path[1:self.n+1] - self.path[0:self.n])/self.T_path - \
						2*velocity_vector[0:self.n] - velocity_vector[1:self.n+1])
		coeff[:, 3] = 1/self.T_path**2*( 2*(- self.path[1:self.n+1] + self.path[0:self.n])/self.T_path + \
						velocity_vector[0:self.n] + velocity_vector[1:self.n+1])

		return coeff 		


	def plot(self, results, method_name, _format='.pdf', _file_path='./results - multi point/'):
		if not os.path.exists(_file_path):
			os.makedirs(_file_path)

		(t, theta, theta_dot, theta_ddot, theta_dddot) = results

		fig = plt.figure()
		fig.set_figheight(15)
		fig.set_figwidth(10)

		# Set global font to Times New Roman
		plt.rc('font', family='Times New Roman')

		if theta_dddot is not None:
			# plot theta
			plt.subplot(411)
			plt.plot(t, theta, linewidth=4)  # Doubled the line thickness
			plt.xlabel(r"$t$", fontsize=28)  # Increased font size
			plt.ylabel(r'$\theta$', fontsize=28)  # Increased font size
			plt.tick_params(axis='both', which='major', labelsize=28)  # Increased axis numbers font size

			# plot theta dot 
			plt.subplot(412)
			plt.plot(t, theta_dot, linewidth=4)  # Doubled the line thickness
			plt.xlabel(r"$t$", fontsize=28)  # Increased font size
			plt.ylabel(r'$\dot{\theta}$', fontsize=28)  # Increased font size
			plt.tick_params(axis='both', which='major', labelsize=28)  # Increased axis numbers font size

			# plot theta double dot 
			plt.subplot(413)
			plt.plot(t, theta_ddot, linewidth=4)  # Doubled the line thickness
			plt.xlabel(r"$t$", fontsize=28)  # Increased font size
			plt.ylabel(r'$\ddot{\theta}$', fontsize=28)  # Increased font size
			plt.tick_params(axis='both', which='major', labelsize=28)  # Increased axis numbers font size

			# plot theta triple dot 
			plt.subplot(414)
			plt.plot(t, theta_dddot, linewidth=4)  # Doubled the line thickness
			plt.xlabel(r"$t$", fontsize=28)  # Increased font size
			plt.ylabel(r'$\dddot{\theta}$', fontsize=28)  # Increased font size
			plt.tick_params(axis='both', which='major', labelsize=28)  # Increased axis numbers font size

		else:
			# plot theta
			plt.subplot(311)
			plt.plot(t, theta, linewidth=4)  # Doubled the line thickness
			plt.xlabel(r"$t$", fontsize=28)  # Increased font size
			plt.ylabel(r'$\theta$', fontsize=28)  # Increased font size
			plt.tick_params(axis='both', which='major', labelsize=28)  # Increased axis numbers font size

			# plot theta dot 
			plt.subplot(312)
			plt.plot(t, theta_dot, linewidth=4)  # Doubled the line thickness
			plt.xlabel(r"$t$", fontsize=28)  # Increased font size
			plt.ylabel(r'$\dot{\theta}$', fontsize=28)  # Increased font size
			plt.tick_params(axis='both', which='major', labelsize=28)  # Increased axis numbers font size

			# plot theta double dot 
			plt.subplot(313)
			plt.plot(t, theta_ddot, linewidth=4)  # Doubled the line thickness
			plt.xlabel(r"$t$", fontsize=28)  # Increased font size
			plt.ylabel(r'$\ddot{\theta}$', fontsize=28)  # Increased font size
			plt.tick_params(axis='both', which='major', labelsize=28)  # Increased axis numbers font size

		plt.tight_layout()
		plt.savefig(_file_path + "mltp - " + method_name + _format)
		plt.clf()


# =================================================================================================
# -- main -----------------------------------------------------------------------------------------
# =================================================================================================

if __name__ == "__main__":

	# Set the path 
	PATH = [0, 1, -1, 0]

	# initialize the path planner class 
	path_planner = PathPlannerMLTP(PATH)

	# point to point methods for multiple points
	results = path_planner.mltp_ptpmethods("ptp_polynomial5th")
	path_planner.plot(results, "ptp polynomial5th")
	# point to point methods for multiple points
	results = path_planner.mltp_ptpmethods("ptp_polynomial7th")
	path_planner.plot(results, "ptp polynomial7th")
	# point to point methods for multiple points
	results = path_planner.mltp_ptpmethods("ptp_polynomial9th")
	path_planner.plot(results, "ptp polynomial9th")
	# point to point methods for multiple points
	results = path_planner.mltp_ptpmethods("ptp_bangbang")
	path_planner.plot(results, "ptp bangbang")
	# point to point methods for multiple points
	results = path_planner.mltp_ptpmethods("ptp_trapezoidal")
	path_planner.plot(results, "ptp trapezoidal")
	# point to point methods for multiple points
	results = path_planner.mltp_ptpmethods("ptp_scurve")
	path_planner.plot(results, "ptp scurve")

	# calculate the trajectory based on cubic spline 
	results = path_planner.mltp_polynomial7th_4point()
	path_planner.plot(results, "7th order polynomial")
	# calculate the trajectory based on cubic spline 
	results = path_planner.mltp_polynomial9th_4point()
	path_planner.plot(results, "9th order polynomial")
	# calculate the trajectory based on cubic spline 
	results = path_planner.mltp_polynomial11th_4point()
	path_planner.plot(results, "11th order polynomial")

	# calculate the trajectory based on cubic spline 
	results = path_planner.mltp_cubicspline()
	path_planner.plot(results, "cubic spline")