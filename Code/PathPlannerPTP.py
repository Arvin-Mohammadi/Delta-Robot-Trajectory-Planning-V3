
# =================================================================================================
# -- imports --------------------------------------------------------------------------------------
# =================================================================================================

from SimpleMath import tand, sind, cosd

import numpy as np 
import math 
import os 
import matplotlib.pyplot as plt 

# =================================================================================================
# -- point to point path planning class -----------------------------------------------------------
# =================================================================================================

class PathPlannerPTP: 
	def __init__(self, theta_initial, theta_final):
		self.theta_i 			= theta_initial
		self.theta_f 			= theta_final
		self.sampling_frequency = 1000


	def ptp_polynomial5th(self):

		# time array (n+1 time instances where n is the sampling frequency)
		t = np.array(range(0, self.sampling_frequency + 1))/self.sampling_frequency

		# calculating s and theta (interpolating polynomial)
		s 			= 6*t**5 - 15*t**4 + 10*t**3
		theta 		= np.array(self.theta_i) + np.array(self.theta_f - self.theta_i)*s

		# calculating s and theta | 1st Differential
		s_dot 		= 30*t**4 - 60*t**3 + 30*t**2
		theta_dot 	= np.array(self.theta_f - self.theta_i)*s_dot

		# calculating s and theta | 2nd Differential
		s_ddot 		= 120*t**3 - 180*t**2 + 60*t
		theta_ddot 	= np.array(self.theta_f - self.theta_i)*s_ddot

		# calculating s and theta | 3rd Differential
		s_dddot 	= -360*t**2 - 360*t + 60
		theta_dddot = np.array(self.theta_f - self.theta_i)*s_dddot

		return(t, theta, theta_dot, theta_ddot, theta_dddot)


	def ptp_polynomial7th(self):

		# time array (n+1 time instances where n is the sampling frequency)
		t = np.array(range(0, self.sampling_frequency + 1))/self.sampling_frequency

		# calculating s and theta (interpolating polynomial)
		s 			= -20*t**7 + 70*t**6 - 84*t**5 + 35*t**4
		theta 		= self.theta_i + (self.theta_f - self.theta_i)*s

		# calculating s and theta | 1st Differential
		s_dot 		= -140*t**6 + 420*t**5 - 420*t**4 + 140*t**3
		theta_dot 	= (self.theta_f - self.theta_i)*s_dot

		# calculating s and theta | 2nd Differential
		s_ddot 		= -840*t**5 + 2100*t**4 - 1680*t**3 + 420*t**2
		theta_ddot 	= (self.theta_f - self.theta_i)*s_ddot

		# calculating s and theta | 3rd Differential
		s_dddot 	= -4200*t**4 + 8400*t**3 - 5040*t**2 + 840*t**1
		theta_dddot = (self.theta_f - self.theta_i)*s_dddot

		return(t, theta, theta_dot, theta_ddot, theta_dddot)


	def ptp_polynomial9th(self):

		# time array (n+1 time instances where n is the sampling frequency)
		t = np.array(range(0, self.sampling_frequency + 1))/self.sampling_frequency

		# calculating s and theta (interpolating polynomial)
		s 			= 70*t**9 - 315*t**8 + 540*t**7 - 420*t**6 + 126*t**5
		theta 		= self.theta_i + (self.theta_f - self.theta_i)*s

		# calculating s and theta | 1st Differential
		s_dot 		= 630*t**8 - 2520*t**7 + 3780*t**6 - 2520*t**5 + 630*t**4
		theta_dot 	= (self.theta_f - self.theta_i)*s_dot

		# calculating s and theta | 2nd Differential
		s_ddot 		= 5040*t**7 - 17640*t**6 + 22680*t**5 - 12600*t**4 + 2520*t**3
		theta_ddot 	= (self.theta_f - self.theta_i)*s_ddot

		# calculating s and theta | 3rd Differential
		s_dddot 	= 35280*t**6 - 105840*t**5 + 113400*t**4 - 50400*t**3 + 7560*t**2
		theta_dddot = (self.theta_f - self.theta_i)*s_dddot

		return(t, theta, theta_dot, theta_ddot, theta_dddot)


	def ptp_bangbang(self):
		# time array (n+1 time instances where n is the sampling frequency)
		t 				= np.array(range(0, self.sampling_frequency + 1)) / self.sampling_frequency
		theta 			= np.zeros_like(t)
		theta_dot 		= np.zeros_like(t)
		theta_ddot 		= np.zeros_like(t)

		for i, time in enumerate(t):
			if time <= 0.5:  		# Phase 1: Constant Acceleration
				theta[i] 		= self.theta_i + 2 * (self.theta_f - self.theta_i) * time**2
				theta_dot[i] 	= 4 * (self.theta_f - self.theta_i) * time
				theta_ddot[i] 	= 4 * (self.theta_f - self.theta_i)

			else:  					# Phase 2: Constant Deceleration
				theta[i] 		= 	0.5 * (self.theta_i + self.theta_f) + 2 * (self.theta_f - self.theta_i) \
									* (time - 0.5) + 2 * (self.theta_i - self.theta_f) * (time - 0.5)**2
				theta_dot[i] 	= 2 * (self.theta_f - self.theta_i) * (1 - 2 * (time - 0.5))
				theta_ddot[i] 	= -4 * (self.theta_f - self.theta_i)

		return (t, theta, theta_dot, theta_ddot, None)


	def ptp_trapezoidal(self):
		
		# time array (n+1 time instances where n is the sampling frequency)
		t = np.array(range(0, self.sampling_frequency + 1)) / self.sampling_frequency
        
		total_time 	= 1  												# total time normalized to 1
		T 			= total_time / 3  									# each phase time duration
		v_max 		= (self.theta_f - self.theta_i) / (total_time - T)  # maximum velocity
		a 			= 3 * v_max  										# acceleration

		theta 		= np.zeros_like(t)
		theta_dot 	= np.zeros_like(t)
		theta_ddot 	= np.zeros_like(t)

		for i, time in enumerate(t):
			if 		time <= T:  						# Phase 1: Acceleration
				theta_dot[i] 	= a * time
				theta_ddot[i] 	= a
				theta[i] 		= self.theta_i + 0.5 * a * time**2
			elif 	time <= 2 * T:  					# Phase 2: Constant Velocity
				theta_dot[i] 	= v_max
				theta_ddot[i] 	= 0
				theta[i] 		= self.theta_i + 0.5 * a * T**2 + v_max * (time - T)
			else:  										# Phase 3: Deceleration
				theta_dot[i] 	= -a * (time - 2 * T) + v_max
				theta_ddot[i] 	= -a
				theta[i] 		= (self.theta_i + 0.5 * a * T**2 + v_max * T + \
                            		v_max * (time - 2 * T) - 0.5 * a * (time - 2 * T)**2)

		return (t, theta, theta_dot, theta_ddot, None)

	def ptp_scurve(self):
		# time array (n+1 time instances where n is the sampling frequency)
		t 				= np.array(range(0, self.sampling_frequency + 1)) / self.sampling_frequency
		total_time 		= 1  	# total time normalized to 1
		T 				= total_time / 7  # each segment time duration

		v_max = (self.theta_f - self.theta_i)/(4*T)
		a_max = v_max/(T*2)
		j_max = a_max/T

		theta 		= np.zeros_like(t)
		theta_dot 	= np.zeros_like(t)
		theta_ddot 	= np.zeros_like(t)
		theta_dddot = np.zeros_like(t)

		for i, time in enumerate(t):
			if time <= T:                       # Phase 1: Increasing Acceleration
				theta_dddot[i]  = j_max
				theta_ddot[i]   = j_max*time
				theta_dot[i]    = 0.5*j_max*time**2
				theta[i]        = (self.theta_i) + 1/6*j_max*time**3
			elif time <= 2 * T:                 # Phase 2: Constant Acceleration
				theta_dddot[i]  = 0
				theta_ddot[i]   = a_max
				theta_dot[i]    = (0.5*a_max*T) + a_max*(time - T)
				theta[i]        = (self.theta_i + (1/12)*v_max*T) + (0.5*a_max*T)*(time - T) + 0.5*a_max*(time - T)**2
			elif time <= 3 * T:                 # Phase 3: Decreasing Acceleration
				theta_dddot[i]  = - j_max
				theta_ddot[i]   = a_max - j_max*(time - 2*T)
				theta_dot[i]    = (1.5*a_max*T) + (a_max*(time - 2*T) - 0.5*j_max*(time - 2*T)**2)
				theta[i]        = (self.theta_i + (7/12)*v_max*T) + (1.5*a_max*T)*(time - 2*T) + \
									(0.5*a_max*(time - 2*T)**2 - (1/6)*j_max*(time - 2*T)**3)
			elif time <= 4 * T:                 # Phase 4: Constant Velocity
				theta_dddot[i]  = 0
				theta_ddot[i]   = 0
				theta_dot[i]    = v_max
				theta[i]        = (self.theta_i + 1.5*v_max*T) + v_max*(time - 3*T)
			elif time <= 5 * T:                 # Phase 5: Increasing Deceleration
				theta_dddot[i]  = - j_max
				theta_ddot[i]   = - j_max*(time - 4*T)
				theta_dot[i]    = v_max - 0.5*j_max*(time - 4*T)**2
				theta[i]        = (self.theta_i + 2.5*v_max*T) + v_max*(time - 4*T) - (1/6)*j_max*(time - 4*T)**3
			elif time <= 6 * T:                 # Phase 6: Constant Deceleration
				theta_dddot[i]  = 0
				theta_ddot[i]   = - a_max
				theta_dot[i]    = (3/4)*v_max - a_max*(time - 5*T)
				theta[i]        = (self.theta_i + (41/12)*v_max*T) + (3/4)*v_max*(time - 5*T) - 0.5*a_max*(time - 5*T)**2 
			else:                               # Phase 7: Decreasing Deceleration
				theta_dddot[i]  = j_max
				theta_ddot[i]   = - a_max + j_max*(time - 6*T)
				theta_dot[i]    = ((1/4)*v_max) + ( - a_max*(time - 6*T) + 0.5*j_max*(time - 6*T)**2)
				theta[i]        = (self.theta_i + (47/12)*v_max*T) + ((1/4)*v_max)*(time - 6*T) + \
									( - 0.5*a_max*(time - 6*T)**2 + (1/6)*j_max*(time - 6*T)**3)
                # this is equal to 4.V_max.T

		return (t, theta, theta_dot, theta_ddot, theta_dddot)

	def plot(self, results, method_name, _format='.png', _file_path='./results - point to point/'):
		if not os.path.exists(_file_path):
			os.makedirs(_file_path)
		
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

	# define path planner 
	path_planner = PathPlannerPTP(0, 1)

	# results for the 5th order polynomial
	results = path_planner.ptp_polynomial5th()
	path_planner.plot(results, "5th order polynomial")

	# results for the 7th order polynomial
	results = path_planner.ptp_polynomial7th()
	path_planner.plot(results, "7th order polynomial")

	# results for the 9th order polynomial
	results = path_planner.ptp_polynomial9th()
	path_planner.plot(results, "9th order polynomial")

    # results for the trapezoidal velocity profile
	results = path_planner.ptp_trapezoidal()
	path_planner.plot(results, "Trapezoidal Velocity Profile")

	# results for the parabolic method
	results = path_planner.ptp_bangbang()
	path_planner.plot(results, "Parabolic Method")

    # results for the S-curve profile
	results = path_planner.ptp_scurve()
	path_planner.plot(results, "S-curve Profile")

