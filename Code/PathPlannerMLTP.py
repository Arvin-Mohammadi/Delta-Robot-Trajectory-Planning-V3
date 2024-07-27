

# =================================================================================================
# -- imports --------------------------------------------------------------------------------------
# =================================================================================================

from SimpleMath import tand, sind, cosd

import os 
import math 
import numpy as np 
import matplotlib.pyplot as plt 
from scipy.interpolate import BSpline, make_interp_spline

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
		self.sampling_number 	= sampling_frequency*self.n + 1
		# time vector corresponding to path points 
		self.t_path				= np.linspace(0, 1, self.n+1)
		# vector of time intervals for the t_path
		self.T_path 			= self.t_path[1:self.n+1] - self.t_path[0:self.n]

	def mltp_bspline(self):
		'''
		Generates a trajectory based on B-spline method for the path
		'''
		k = 3  # degree of the spline (cubic B-spline)
		t = np.linspace(0, 1, self.sampling_number)

		# Generate a B-spline representation of the curve
		spl 	= make_interp_spline(self.t_path, self.path, k=k)
		theta 	= spl(t)

		# Compute derivatives of the B-spline
		theta_dot 		= spl.derivative(nu=1)(t)
		theta_ddot 		= spl.derivative(nu=2)(t)
		theta_dddot 	= spl.derivative(nu=3)(t)

		return (t, theta, theta_dot, theta_ddot, theta_dddot)

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


	def plot(self, results, method_name, _num_differentials=3, _format='.png', _file_path='./results - multi point/'):
		if not os.path.exists(_file_path):
			os.makedirs(_file_path)
		
		if _num_differentials == 3:
			(t, theta, theta_dot, theta_ddot, theta_dddot) = results

			fig = plt.figure()
			fig.set_figheight(15)
			fig.set_figwidth(10)

			# plot theta
			plt.subplot(411)
			plt.plot(t, theta, label=r'$\theta$')
			plt.legend()
			plt.title(r'value-time', fontsize=20)
			plt.xlabel("time", fontsize=15)
			plt.ylabel("value", fontsize=15)


			# plot theta dot 
			plt.subplot(412)
			plt.plot(t, theta_dot, label=r'$\dot{\theta}$')
			plt.legend()
			plt.title(r'first differential value-time', fontsize=20)
			plt.xlabel("time", fontsize=15)
			plt.ylabel("first differential value", fontsize=15)

			# plot theta double dot 
			plt.subplot(413)
			plt.plot(t, theta_ddot, label=r'$\ddot{\theta}$')
			plt.legend()
			plt.title(r'second differential value-time', fontsize=20)
			plt.xlabel("time", fontsize=15)
			plt.ylabel("second differential value", fontsize=15)

			# plot theta triple dot 
			plt.subplot(414)
			plt.plot(t, theta_dddot, label=r'$\dddot{\theta}$')
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
	PATH = [0, -0.2, 0.3, 0.8, -0.1, 1]

	# initialize the path planner class 
	path_planner = PathPlannerMLTP(PATH)

	# calculate the trajectory based on cubic spline 
	results = path_planner.mltp_cubicspline()
	path_planner.plot(results, "cubic spline")

	# calculate the trajectory based on B spline 
	results = path_planner.mltp_bspline()
	path_planner.plot(results, "B spline")
