# Trajectory Planning Experimental Comparison Study

<img align="right" src="https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/5d0b34e0-8cbd-4d3d-9884-382a565008ef" width=25%>

Overview: 
- [Introduction](#section-introduction)
- [Delta Robot Kinematics](#section-deltarobot_kinematics)
- [Theoretical Study - Point-to-Point Trajectory Generation](#section-point2point_trajectory_generation)
- [Theoretical Study - Multi-Point Trajectory Generation](#section-multipoint_trajectory_generation)
- [Adept Cycle](#section-adeptcycle) 
- [References](#section-references)
- [Ending Note](#section-endnote)

</br>


<a name="section-introduction"></a>
## Introduction

<ins>**What is a Delta Robot and Why is it Important**</ins> 

People who search for this repository already know what the delta robot is and know why it's important. Delta robot parallel kinematic structure and high-speed capabilities make them ideal for precise and speedy tasks, particularly in pick-and-place operations. This repository studies trajectory planning methods for Delta robots, focusing on smooth motion for the end-effector while minimizing deviations.

**The applications of Delta robots** are virtually endless but mainly they come down to a certain pattern of pick-and-place or 3D printing or tasks that require somewhat similar movements to these two. Here's three applications that I worked on (PLEASE CITE MY 4 PAPERS):

- Pick-and-Place Operations: [LINK](https://ieeexplore.ieee.org/abstract/document/10334699)
- Automated Pipetting Operation: [LINK](https://ieeexplore.ieee.org/abstract/document/10412424)
- Food Packaging - Not Published Yet
- Automated Pipetting Operation: Not Published Yet

<ins>**Pros and Cons of Delta Robot**</ins>

Delta robot has basically two advantages at the cost of two things (I'm talking about the important ones). 

Advantages: 
1. High Speed
2. High Precision

Disadvantages:
1. Small Workspace
2. Small Workload
<img align="right" src="https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/5924887e-6c97-4af0-b3d5-d6d9a3c5c459" width=30%>


<ins>**Trajectory Planning**</ins> 

Take a look at the figure below. There are four main stages to any sort of robotic operation:
1. **Task Planning:** Figuring out what the robot is going to be doing [in order]
2. **Path Planning:** What points in 3D space the robot is going through [in order]
3. **Trajectory Planning:** Positon as a function of time
4. **Control**: Giving the trajectory as a reference to the robot's controller

</br>

<a name="section-deltarobot_kinematics"></a>
## Delta Robot Kinematics
<img align="right" src="https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/a06639cb-afbb-47c5-8a0d-340a20674f84" width=30%>

<ins>**What are Forward and Inverse Kinematics**</ins> 

Look at the figure below. Let's say our robot has actuated joints of $\[\theta_1, \theta_2, \theta_3\]$ and the position of the end-effector is $\[x, y, z\]$:

- **Forward Kinematics:** Given the actuated joint parameters to calculate the position of end-effector
- **Inverse Kinematics:** Given the position of the end-effector to calculate the actuated joint parameters

<ins>**What is the Jacobian of a Robot**</ins> 

This basically has the same logic as the FK and IK but this time instead of converting between positions and angles, the conversion occurs between velocity of the end-effector and the velocity of the joint parameters. 


</br>

### Theoretical Solution of Forward and Inverse Kinematics in Delta Robot

<div align="center">
 	<img src="https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/b34332cf-6016-42b2-83ac-fc2824447b97" style="width: 50%;">
	</br>
	_Note: The solution is from the reference #1_
</div>

</br>


<ins>**Given Data & Assumptions**</ins> 

The Delta robot consists of three main chains. Each chain starts from the base platform($O_0$), connects to the upper arm via the pin joint ($A_i$), connects to the lower arm via the universal joint ($B_i$), and finally connects to the end-effector via the universal joint ($C_i$). This results in a movement behaviour, where the end-effector moves parallel to the base platform and can move along three translational axes of ($x, y, z$) in 3D space. 

```math
\begin{cases} 
  R_i & \equiv \overline{O_0A_i} & = \text{The distance between base platform center and the pin joint} \\
  r_i & \equiv \overline{O_PC_i} & = \text{The distance between the end-effector center and the universal joint $C_i$} \\
  L_i & \equiv \overline{A_iB_i} & = \text{The length of each actuated link (upper arm)}  \\
  l_i & \equiv \overline{B_iC_i} & = \text{The length of each driven link (lower arm)}
\end{cases}
```

The angles between $x_0$ and $\overline{O_0A_i}$ are indiacted by $\gamma_i$. The angles between $x_p$ and $\overline{O_PC_i}$ are indicated by $\beta_i$. The angles between the actuaated links and the horizon are indicated by $\theta_i$, where $i=1, 2, 3$

We have the following assumptions:

```math
\begin{cases}
  L & = L_i \\
  l & = l_i \\
  R & = R_i \\
  r & = r_i
\end{cases}
```

and also: 

```math
\gamma_1 = \frac{1}{2}\gamma_2 = \frac{1}{3}\gamma_3 = \beta_1 = \frac{1}{2}\beta_2 = \frac{1}{3}\beta_3 = 120\degree
```

<ins>**Calculating Relative Positions**</ins> 

- The position of $A_i$ in relation to the $\{O_0\}$-frame:

```math
    \overline{O_0A_i} = [R\cos\gamma_i \quad R\sin\gamma_i \quad 0]^T
```

- The position of $B_i$ in relation to the $\{O_0\}$-frame:

```math
    \overline{O_0B_i} = 
    \begin{bmatrix}
        (R + L\cos\theta_i)\cos\gamma_i \\ 
        (R + L\cos\theta_i)\sin\gamma_i \\
        - L\sin\theta_i 
    \end{bmatrix}
```

- The position of $C_i$ in relation to the $\{O_0\}$-frame:

```math
    \overline{O_PC_i} = [r\cos\gamma_i \quad r\sin\gamma_i \quad 0]^T
```

-  The position of $C_i$ in relation to the $\{O_0\}$-frame, given that the position of end-effector center is equal to $[X_P \quad Y_P \quad Z_P]^T$:

```math
    \overline{O_0C_i} = 
    \begin{bmatrix}
        X_P + r\cos\gamma_i\\ 
        Y_P + r\sin\gamma_i \\ 
        Z_P
    \end{bmatrix}
```

- Given the mentioned equations we can say that we have the following \textbf{constraint}:

```math
    l = |\overline{O_0B_i} - \overline{O_0C_i}|
```
- Re-writing the previous equation we have: 

```math 
    \overline{O_0B_i - O_0C_i} = 
    \begin{bmatrix}
        (R - r + L\cos\theta_i)\cos\gamma_i - X_P \\ 
        (R - r + L\cos\theta_i)\sin\gamma_i - Y_P \\ 
        - L\sin\theta_i - Z_P
    \end{bmatrix}
```

<ins>**Solution of Foward and Inverse Kinematics**</ins> 

1. For FK we numerically solve the above constraint equation for $(X_P, Y_P, Z_P)$ given $\theta_i$

2. For IK we assume a variable change of $t_i = \tan(\theta_i/2)$, which gives us $\sin(\theta_i) = \frac{2t}{t^2+1}$ and $\cos(\theta_i) = \frac{1 - t^2}{t^2+1}$. Applying this, we solve the constraint equation for $t_i$, which in turn, gives us $\theta_i$.




<a name="section-point2point_trajectory_generation"></a>
### Experimental Solution of Forward and Inverse Kinematics in Delta Robot

If you need a plug and place code that **JUST WORKS** i suggest the following code: [LINK](https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/blob/main/References/Inverse%20Kinematics%20(Delta%20Robot).pdf) - Reference #2


<a name="subsection-point2point_trajectory_generation"></a>
## Theoretical Study - Point-to-Point Trajectory Generation

Basically point-to-point trajectory planning is like interpolation between two values (let's call them $\theta^I$ and $\theta^F$), and to us it outputs an interpolation of these two values as a function of time; then we take that function and sample it at a constant sampling frequency, take the resulting array of values and give that array to the robot's PID controller. 

```math
  \begin{cases}
    & \Theta^I & = \text{Initial Value} \\
    & \Theta^F & = \text{Final Value}
  \end{cases}
```

<ins>**NOTE**</ins>: The output array does not include any time information, the time information comes into play when we want to give the PID controller the next target point from the array. So basically when the robot wants to move from Point A to  B in either 2 seconds or 3 seconds, the interpolation array for both of these actions are the same. 

<ins>**NOTE**</ins>: Since the array does not include time information, the duration for the whole process is considered to be 1, hence it simplifies a lot of the calculations. We call this "normalized time". If you need the time information included you should refer to the main references. 

### Bang-Bang 

We need to define the main phases of movement at the start of each method, so for this the important time instances are:

```math
\begin{aligned}
  t_0 & \equiv \text{Positive acceleration phase starts} \\ 
  t_1 & \equiv \text{Positive acceleration phase ends | Negative acceleration phase starts} \\ 
  t_2 & \equiv \text{Negative acceleration phase ends} 
\end{aligned}
```
So the trajectory is also called the parabolic trajectory and is actually made of two 2nd order polynomials glued together, in mathmetical form that will look like: 

```math
\begin{aligned}
	\theta_a(t) & = a_0 + a_1(t - t_0) + a_2(t - t_0)^2 & \quad\text{for}‌\quad t_0 \leq t \leq t_1 \\ 
	\theta_b(t) & = a_3 + a_4(t - t_1) + a_5(t - t_1)^2 & \quad\text{for}‌\quad t_1 \leq t \leq t_2 \\ 
\end{aligned}
```

What we can do is to set $t_0 = 0, t_1 = 0.5, t_2 = 1$ and then simplify everything to reach:  

```math
\begin{aligned}
	\theta_a(t) & = a_0 + a_1t + a_2t^2 & \quad\text{for}‌\quad 0 \leq t \leq 0.5 \\ 
	\theta_b(t) & = a_3 + a_4(t - 0.5) + a_5(t - 0.5)^2 & \quad\text{for}‌\quad 0.5 \leq t \leq 1 \\ 
\end{aligned}
```

Then we write down the conditions for solving $\theta_a(t)$. The conditions are:
1. At the start of movement in this phase, $\theta_a(t)$ is equal to $\theta^I$
2. At the end of the movement in this phase, $\theta_a$ is exactly half-way between $\theta^I$ and $\theta^F$
3. At the start of movement in this phase, velocity (1st differential) of $\theta_a(t)$ is equal to zero
which in mathematical form is written as:

```math
\begin{aligned}
	\text{conditions for solving $\theta_a(t)$} \rightarrow
	& \theta_a(t=0) = \theta^I = a_0 \\ 
	&‌ \theta_a(t=0.5) = \frac{\theta^I + \theta^F}{2} = a_0 + a_1(0.5) + a_2(0.5)^2 \\ 
	&‌ \dot{\theta}_a(t=0) = 0 = a_1 
\end{aligned}
```

Solving the system of linear equations we get:

```math
	a_0 = \theta^I, \quad a_1 = 0, \quad a_2 = 2(\theta^F - \theta^I)
```

Then we write down the conditions for solving $\theta_a(t)$. The conditions are:
1. At the start of movement in this phase $\theta_b(t)$ is exactly half-way between $\theta^I$ and $\theta^F$
2. At the end of the movement in this phase, $\theta_b(t)$ is equal to $\theta^F$
3. At the end of movement velocity (1st differential) of $\theta_b(t)$ is equal to zero
which in mathematical form is written as:


```math
\begin{aligned}
	\text{conditions for solving $\theta_b$} \rightarrow
	& \theta_b(t=0.5) = \frac{\theta^I+\theta^F}{2} = a_3 \\ 
	& \theta_b(t=1) = \theta^F = a_3 + a_4(0.5) + a_5(0.5)^2 \\ 
	&‌ \dot{\theta}_b(t=1) = 0 = a_4 + 2a_5(0.5)
\end{aligned}
```


Solving the system of linear equations we get:

```math
	a_3 = \frac{\theta^I+\theta^F}{2}, \quad a_4 = 2(\theta^F - \theta^I), \quad a_5 = 2(\theta^I - \theta^F)
```

Finally the overall mathematicall function can be described as:

```math
\begin{aligned}
	\theta_a(t) & = \theta^I + 2(\theta^F - \theta^I)t^2 &‌ \quad\text{for}\quad 0 \leq t \leq 0.5 \\ 
	\theta_b(t) & = \frac{\theta^I+\theta^F}{2} + 2(\theta^F - \theta^I)(t - 0.5) + 2(\theta^I - \theta^F)(t - 0.5)^2 &‌ \quad\text{for}\quad 0.5 \leq t \leq 1
\end{aligned}
```

<div align="center">
 	<img src="https://github.com/user-attachments/assets/12e85bb4-e6d7-4b7a-83e4-7890e3506a0f" style="width: 50%;">
	</br>
	Bang-Bang Method | Parabolic Method
</div>
</br>

### Trapezoidal 

As explained, the goal here is to basically use a trapezoidal diagram as a way to interpolate between two given motor rotations. The trapezoidal diagram is defined as the following:

```math
  \dot{\theta} = 
  \begin{cases}
    at        & t_0 \leq t < t_1 \\
    V_{max}   & t_1 \leq t < t_2  \\
    -at       & t_2 \leq t \leq t_3
  \end{cases}
```

For the sake of simplicity we say that $t_0 = 0, t_1 = 1/3, t_2 = 2/3, t_3 = 1$. Using this we can also conclude that $v_{max} = a.t$ where $t=1/3$ and this results in  $a = 3v_{max}$. Finally we have: 
```math
  \dot{\theta} = 
  \begin{cases}
    3V_{max}t        & 0 \leq t < 1/3 \\
    V_{max}   & 1/3 \leq t < 2/3  \\
    -3V_{max}t       & 2/3 \leq t \leq 1
  \end{cases}
```


<div align="center">
 	<img src="https://github.com/user-attachments/assets/56500e38-c4fc-462e-a887-dea3079504c5" style="width: 50%;">
	</br>
	Trapezoidal Method
</div>
</br>


### S-curve

The S-curve method is somewhat similar to the trapezoidal method with the difference that it is smoother. So in mathemtical terms that would be: 

```math
\begin{aligned}
	\ddot{\theta} = 
	\begin{cases}
		j.t         & \quad\text{for}\quad t_0 \leq t < t_1 \\
		a_{max} 	& \quad\text{for}\quad t_1 \leq t < t_2 \\
		-j.t 		& \quad\text{for}\quad t_2 \leq t < t_3 \\
		0 			& \quad\text{for}\quad t_3 \leq t < t_4 \\
		-j.t		& \quad\text{for}\quad t_4 \leq t < t_5 \\
		-a_{max}	& \quad\text{for}\quad t_5 \leq t < t_6 \\
		j.t			& \quad\text{for}\quad t_6 \leq t \leq t_7
	\end{cases}
\end{aligned}
```


<div align="center">
 	<img src="https://github.com/user-attachments/assets/70db62d1-d776-4765-a389-68538a438e0e" style="width: 50%;">
	</br>
	S-Curve Method
</div>
</br>

### Interpolating Polynomials


When interpolating between $\theta^I$ and $\theta^F$ we can use a polynomial such as $s(t)$ where $0 \leq t \leq 1$ since the time is normalized in this context and also $0 \leq s \leq 1$. And then you relate $s(t)$ with $\theta(t)$ as written below:

```math
	\theta(t) = \theta^I + (\theta^F - \theta^I)s(t)
```

Now knowing that, we can talk about $s(t)$ and we are gonna use the 5th, 7th, and 9th degree polynomials for this. 

#### 5th order polynomial 

The $s(t)$ polynomial can be written as: 

```math
	s(t) = a.t^5 + b.t^4 + c.t^3 + d.t^2 + e.t + f
```

we can write the conditions to solve for $a, b, c, d, e, f$:
1. starting and finishing position of $s$ which are 0 and 1
2. starting and finishing velocity of $s$ which are both 0
3. starting and finishing acceleration of $s$ which are both 0
These conditions can be written in mathmetical format:  

```math
\begin{aligned}
	s(0) = 0, \quad \dot{s}(0) = 0, \quad \ddot{s}(0) = 0 \\ 
	s(1) = 1, \quad \dot{s}(1) = 0, \quad \ddot{s}(1) = 0
\end{aligned}
```

Solving for the resulting system of linear equations we get: 

```math
	a = 6, \quad b = -15, \quad c = 10, \quad d=e=f=0
```


and in the end we have:

```math
	s(t) = 6.t^5 - 15.t^4 + 10.t^3
```

which means:

```math
	\theta(t) = \theta^I + (\theta^F - \theta^I)(6.t^5 - 15.t^4 + 10.t^3)
```

<div align="center">
 	<img src="https://github.com/user-attachments/assets/dee8093b-4d78-4f55-90bd-c46c7d0e57ac" style="width: 50%;">
	</br>
	5th-order polynomial
</div>
</br>


#### 7th order polynomial


We go through the same process of as before in the 5th order polynomial. First it is to write down $s(t)$:

```math
	s(t) = a.t^7 + b.t^6 + c.t^5 + d.t^4 + e.t^3 + f.t^2 + g.t + h
```

then writing down the conditions:

```math
	\begin{aligned}
		s(0) = 0, \quad \dot{s}(0) = 0, \quad \ddot{s}(0) = 0, \quad \dddot{s}(0) = 0 \\ 
		s(1) = 1, \quad \dot{s}(1) = 0, \quad \ddot{s}(1) = 0, \quad \dddot{s}(1) = 0
	\end{aligned}
```

Then solving the linear system of equations based on the conditions:

```math
	a=-20, \quad b = 70, \quad c = -84, \quad d = 35, \quad e=f=g=h=0
```

and at the end we can write:

```math
	s(t) = -20.t^7 + 70.t^6 - 84.t^5 + 35.t^4
```

which means for $\theta(t)$ we have:
```math
	\theta(t) = \theta^I + (\theta^F - \theta^I)(-20.t^7 + 70.t^6 - 84.t^5 + 35.t^4)
```

<div align="center">
 	<img src="https://github.com/user-attachments/assets/67bb3d24-3a9c-4c92-9cbf-19afd13afd12" style="width: 50%;">
	</br>
	7th-order polynomial
</div>
</br>


#### 9th order polynomial

Again the same shit is both 5th and 7th order polynomial. First $s(t)$:

```math
	a_1.t^9 + a_2.t^8 + \dots + a_8.t^2 + a_9.t + a-{10}
```

then writing down the conditions:

```math
	\begin{aligned}
		s(0) = 0, \quad \dot{s}(0) = 0, \quad \ddot{s}(0) = 0, \quad \dddot{s}(0) = 0, \quad \ddot{\ddot{s}}(0) = 0 \\ 
		s(1) = 1, \quad \dot{s}(1) = 0, \quad \ddot{s}(1) = 0, \quad \dddot{s}(1) = 0, \quad \ddot{\ddot{s}}(1) = 0
	\end{aligned}
```

Then solving the system of linear equations:

```math
\begin{aligned}
	a_1 & = 70, \quad a_2 = -315, \quad a_3 = 540, \quad a_4 = -420 \\\
	a_5 & = 126, \quad a_6=a_7=a_8=a_9=a_{10}=0
\end{aligned}
```

So we have for $s(t)$:

```math
	s(t) = 70.t^9 - 315.t^8 + 540.t^7 - 420.t^6 + 126.t^5
```

and we have for $\theta(t)$: 

```math
	\theta(t) = \theta^I + (\theta^F - \theta^I)(70.t^9 - 315.t^8 + 540.t^7 - 420.t^6 + 126.t^5)
```


<div align="center">
 	<img src="https://github.com/user-attachments/assets/c8ad43ab-9485-4e7c-bc79-4aa5d9a46071" style="width: 50%;">
	</br>
	9th-order polynomial
</div>
</br>

### Discussion 

So we took a look at six different methods:

1. Bang-Bang | Parabolic Trajectory
2. Trapezoidal Velocity Trajectory
3. S-Curve Velocity Trajectory
4. 5th Order Interpolating Polynomial
5. 7th Order Interpolating Polynomial
6. 9th Order Interpolating Polynomial

All of them are usable, but from my experience for point-to-point movement the polynomials are generally better and what's more, 7th order polynomial is probably the best one. Why is that? Well firstly it's kind of hard for the PID controller to follow trajectories such as trapezoidal (for some reason) rather than the polynomials (maybe because they are smoother in relation to the other three methods). But then why choose 7th order? well because it controls the jerk (unlike the 5th order). The 9th order polynomial ain't bad either. it controls the snap at the start and finish as well as the jerk. 

So yeah from my experience the best ones you can implement for your robot ... whatever it is ... is probably the 7th or 9th order polynomial. It's easy to calculate, easy to implement, reliable, and also the PID controller won't have a problem following it. 

<a name="section-multipoint_trajectory_generation"></a>
## Theoretical Study - Multi-Point Trajectory Generation

Multi-point trajectory generation generates a trajectory between multiple target points given as a path, along with its time information. For example if the robot needs to be moved from point A, passes point B and then stop at point C (same logic with more points, basically we have a path which contains more than the starting and final points).

So like the point-to-point methods the **inputs** are the path points which want to hit, and then the **output** is the array of values (sampled from the position as a function of time) that we'll give to the PID Controller as a reference. 

### Bad Choices for Multi-Point Trajectory

Probably the worst methods you can choose for a multiple-point trajectory planning are either using a high-order polynomial interpolation or one of the point-to-point methods. Let's elaborate further. 

#### High-Order Polynomial Interpolation

Assume you have a set of values such as [0, 0.2, 1] and you wanted to create a trajectory for these values with a polynomial:

```math
	s(t) = a_n.t^n + \dots + a_1.t + a_0
```

So given this information we list the conditions for creating the polynomial function: 

1. The three positions of [0, 0.2, 1]: Conditions x3
2. Initial and final velocity values [0, 0]: Conditions x2
3. Initial and final acceleration values [0, 0]: Conditions x2
4. Initial and final jerk values [0, 0]: Conditions x2

So that gives us 9 conditions in total. For 9 conditions we need an 8th-order polynomial to be able to create a linear system of equations for the coefficients. Meaning the polynomial would be: 

```math
	s(t) = a_8.t^8 + \dots + a_1.t + a_0
```

And the equations will be something like: 

```math
\begin{aligned}
	s(0) 		& = 0, \quad s(0.2) 		= 0.2, \quad s(1) = 1 \\
	\dot{s}(0) 	& = 0, \quad \dot{s}(1) 	= 0 \\
	\ddot{s}(0) 	& = 0, \quad \ddot{s}(1) 	= 0 \\
	\dddot{s}(0) 	& = 0. \quad \dddot{s}(1) 	= 0 
\end{aligned}
```

Solving this linear system of equations will result in the values of $a_8, \dots, a_0$. And it works. But why is it a bad choice? Simply because it can't be generalized. Meaning that if we have 4 points of interest that we want to cross (so instead of [0, 0.2, 1] we would have [0, -0.4, 0.2, 1]) then the calculations we just did will completely fall apart. So there's that. It works ... but isn't practical. Don't use this method.

#### Point-to-Point Methods 

Firstly how can we use a point-to-point method for multiple number of points? It's easy. Let's go with the previous example and say that we want to create a trajectory for the three values of [0, 0.2, 1]. First, We can simply create a trajectory between [0, 0.2] and then create another trajectory between [0.2, 1]. This can be done with any number of points and we can use all of the 6 mentioned point-to-point methods. Why is it a bad idea? Because the velocity and acceleration will be zero at all of the points. So assume we have a number of points that we want our robots to cross over such as $P_0, P_1, P_2, P_3, \dots, P_n$. Given that we use the method I just explained, the robot will stop at all of the points of $P_0$ to $P_n$ and start moving again ... sort of like the robot has got the hiccups, which is awful for the motors. Don't use this methods either. 

### Cubic Spline 

This method basically uses multiple polynomials on end for generating a trajectory between multiple path points. So if we are given $n+1$ path points, we'll need $n$ polynomials to interpolate and create the trajectory. 

Each polynomial will have a degree of $p$ based on the desired level of trajectory smoothness. As default we'll have $p=3$ which will give us a smooth velocity profile and a continuous acceleration profile

So the input is:

```math
\begin{aligned}
	\text{path array} = [\theta_0, \theta_1, \dots, \theta_n, \theta_{n+1}]   
\end{aligned}
```

The overall trajectory function can be described as: 

```math
\begin{aligned}
	\theta(t) & = \lbrace q_k(t), t \in [t_k, t_{k+1}], k=0, ..., n \rbrace \\
	\text{where} \quad q_k(t) & = a_{k0} + a_{k1}(t-t_k) + a_{k2}(t-t_k)^2 + a_{k3}(t-t_k)^3
\end{aligned}
```

So as I said there are $n$ polynomials with unknown coefficients that connect every two points in the given path. So we need to calculate the value of $4n$ coefficients. We calculate the values of coefficients based on some conditions such as: 

- Adhering to the path values (at the starting and finishing point of each polynomial) x(2n) Conditions
-  The velocity should be continuous at the transition points between each two successive polynomials (such as polynomial number $k$ and $k+1$) x(n-1) Conditions
-  The acceleration should be continuous at the transition points between each two successive polynomials (such as polynomial number $k$ and $k+1$) x(n-1) Conditions
-  Initial and final velocity of the movement x2 Conditions 

Adding these conditions up will result in 4n conditions, hence 4n equations. Solving this system of linear equations will give us the result. Let's write the conditions one more time: 

```math
\begin{aligned}
	& q_k(t_k) = \theta_k, \quad q_k(t_{k+1}) = \theta_{k+1}, & k=0, ..., n \\
	& \dot{q_k} (t_{k+1}) 	= \dot{q_{k+1}}(t_{k+1}), & k=0, ..., n-1\\
	& \ddot{q_k} (t_{k+1}) 	= \ddot{q_{k+1}} (t_{k+1}), & k=0, ..., n-1\\
	& \dot{q_0} (t_0) 	= 0 , \quad \dot{q_{n-1}} (t_n) = 0 & \\
\end{aligned}
```

The coefficient $a_{k,i}$ can be computed with the following steps. First we consider each velocity at time $t_k$ to be known:

```math
\begin{aligned}
	q_k(t_k) & = a_{k0}						& = q_k \\
	\dot{q_k} (t_k) & = a_{k1} 					& = v_k \\ 
	q_k(t_{k+1}) & = a_{k0} + a_{k1} T_k + a_{k2} T^2_k + a_{k3}T^3_k & = q_{k+1} \\ 
	\dot{q_k} (t_{k+1}) & = a_{k1} + 2a_{k2} T_k + 3 a_{k3} T^2_k 	& = v_{k+1} \\ 
\end{aligned}
```

Where $T_k = t_{k+1} - t_k$. Solving the above equations we have: 

$$
\begin{cases}
    a_{k,0} & = q_k\\ 
    a_{k,1} & = v_k\\ 
    a_{k,2} & = \frac{1}{T_k}   [\frac{3(q_{k+1} - q_k)}{T_k} - 2v_k - v_{k+1}] \\ 
    a_{k,3} & = \frac{1}{T^2_k} [\frac{2(q_k - q_{k+1})}{T_k} + v_k + v_{k+1}] \\
\end{cases}
$$


But this is for when the velocities of the points are known, which they are not (except the initial and final points). So the velocities have to be calculated, in this instance we use the continuity conditions of acceleration. Velocities can be found with a matrix of $v = A^{-1}c$. Where: 


$$
A = 
\begin{bmatrix}
    2(T_0+T_1)     & T_0         & 0       & ...                             &     & 0 \\
    T_2            & 2(T_1+T_2)  & T_1     & 0                               &     & \vdots \\
    0              &             & \ddots  &                                 &     & 0 \\
    \vdots         &             &         & T_{n-1}  & 2(T_{n-2}+T_{n-1})   & T_{n-2} \\ 
    0              & \dots       &         & 0        & T_{n}              & 2(T_{n-1} + T_{n}) \\  
\end{bmatrix}
$$

$$
c = 
\begin{bmatrix}
    \frac{3}{T_0T_1} \left[ T^2_0(q_2 - q_1) + T^2_1(q_1 - q_0) \right] - T_1 v_0 \\
    \frac{3}{T_1T_2} \left[ T^2_1(q_3 - q_2) + T^2_2(q_2 - q_1) \right] \\
    \vdots \\ 
    \frac{3}{T_{n-2}T_{n-1}} \left[ T^2_{n-2}(q_{n} - q_{n-1}) + T^2_{n-1}(q_{n-1} - q_{n-2}) \right] \\
    \frac{3}{T_{n-1}T_{n}} \left[ T^2_{n-1}(q_{n+1} - q_{n}) + T^2_{n}(q_{n} - q_{n-1}) \right] - T_{n-1}v_{n+1} \\
\end{bmatrix}
$$

$$
v = 
\begin{bmatrix}
    v_1 \\ 
    v_2 \\ 
    \vdots \\ 
    v_{n-1} \\ 
    v_{n} \\ 
\end{bmatrix}
$$

For better understanding please refer to [3] in the reference section. 

### B-Spline

### Pattern Generation 

<a name="section-code_review"></a>
## Code Review

In this section I'll explain each of the code files and how to use them. Firstly there is a ```SimpleMath.py``` file which you don't have to worry about too much since it only introduces functions of ```sind```, ```cosd```, and ```tand``` which are based on their corresponding numpy functions with the difference of getting inputs in degrees rather than radians. 

### PathPlannerPTP.py

This file introduces a point-to-point path planner class and for initiating the class you can write:

```python
THETA_I = 0 # intial value 
THETA_F = 1 # final value 
path_planner = PathPlannerPTP(THETA_I, THETA_F)
```

As discussed in the previous sections there are 6 point to point methods implemented: 

1. Parabolic | Bang-Bang Method
2. Trapezoidal Velocity Method
3. S-Curve Velocity Method
4. 5th-Order Interpolating Polynomial
5. 7th-Order Interpolating Polynomial
6. 9th-Order Interpolating Polynomial 

You can call each of those methods and plot the results like this:

```python
# results for the parabolic method
results = path_planner.ptp_bangbang()
path_planner.plot(results, "Parabolic Method", num_differentials=2)

# results for the trapezoidal velocity profile
results = path_planner.ptp_trapezoidal()
path_planner.plot(results, "Trapezoidal Velocity Profile", num_differentials=2)

# results for the S-curve velocity profile
results = path_planner.ptp_scurve()
path_planner.plot(results, "S-curve Profile")

# results for the 5th order interpolating polynomial
results = path_planner.ptp_polynomial5th()
path_planner.plot(results, "5th order polynomial")

# results for the 7th order interpolating polynomial
results = path_planner.ptp_polynomial7th()
path_planner.plot(results, "7th order polynomial")

# results for the 9th order interpolating polynomial
results = path_planner.ptp_polynomial9th()
path_planner.plot(results, "9th order polynomial")
```

in each of the different methods used, the ```results``` is always an array of values that interpolates the initial and final values 

<a name="section-adeptcycle"></a>
## Adept Cycle 


<a name="section-references"></a>
## References

1. [Kinematic Analysis of Delta Parallel Robot: Simulation Study - A. Eltayeb](https://www.researchgate.net/publication/352787189_Kinematic_Analysis_of_Delta_Parallel_Robot_Simulation_Study) 
2. [?](https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/blob/main/References/Inverse%20Kinematics%20(Delta%20Robot).pdf)
3. [Trajectory Planning for Automatic Machines and Robots - Luigi Biagiotti](https://link.springer.com/book/10.1007/978-3-540-85629-0)
4. [Fundamentals of Robotic Mechanical Systems: Theory, Methods, and Algorithms - Jorge Angeles](https://link.springer.com/book/10.1007/978-3-319-01851-5)
5. 

<a name="section-endnote"></a>
## Ending Note

Ok real talk here. I'm absolutely sick of working on papers and things like that. I think they are pretentious and faily useless for computer and robotics projects such as this one. I think github projects are millions of times more valuable because they provide real insight and tend to lean on explaining how things work instead of trying to convince you why the work is really important. But I need to work on publishing papers and also finishing my bachelor thesis ... at least for now. But just know that sharing this information in the raw format and purely practical way like this with the people around the world who might need it, is what makes me even slightly excited about doing this. that's what keeps me going. so thanks for taking interest in the previous projects I did on Delta robot trajectory planning. this will be the final version and I will hopefully never work or even see another delta robot in my entire life also follow me on instagram if you're into gaming 🖥️🎮🤖  

