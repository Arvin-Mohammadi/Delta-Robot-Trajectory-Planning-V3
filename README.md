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


<ins>**Trajectory Planning**</ins> 

Take a look at the figure below. There are four main stages to any sort of robotic operation:
1. **Task Planning:** Figuring out what the robot is going to be doing [in order]
2. **Path Planning:** What points in 3D space the robot is going through [in order]
3. **Trajectory Planning:** Position of the robot as a function of time
4. **Control**: Giving the trajectory as a reference to the robot's controller

</br>
<div align="center">
 	<img src="https://github.com/user-attachments/assets/5aab2c7c-5e96-4978-9880-9c58a5b184c5" style="width: 50%;">
	</br>
	Simple robotics tasks are carried out as the depicted model.
</div>
</br>

<a name="section-deltarobot_kinematics"></a>
## Delta Robot Kinematics

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
	Note: The solution is from the reference #1
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

The angles between $x_0$ and $\overline{O_0A_i}$ are indiacted by $\gamma_i$. The angles between $x_p$ and $\overline{O_PC_i}$ are indicated by $\beta_i$. The angles between the actuated links and the horizon are indicated by $\theta_i$, where $i=1, 2, 3$

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

- Given the mentioned equations we can say that we have the following constraint:

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


<div align="center">
 	<img src="https://github.com/user-attachments/assets/9ce531d2-bc5e-4fe1-b94d-7343e57593a2" style="width: 50%;">
	</br>
	Point-to-Point movement
</div>
</br>

Basically point-to-point trajectory planning is like interpolation between two values (let's call them $\theta^I$ and $\theta^F$), and to us it outputs an interpolation of these two values as a function of time; then we take that function and sample it at a constant sampling frequency, take the resulting array of values and give that array to the robot's PID controller. 

```math
  \begin{cases}
    & \Theta^I & = \text{Initial Value} \\
    & \Theta^F & = \text{Final Value}
  \end{cases}
```

<ins>**NOTE**</ins>: The output array does not include any time information, the time information comes into play when we want to give the PID controller the next target point from the array. So basically when the robot wants to move from Point A to  B in either 2 seconds or 3 seconds, the interpolation array for both of these actions are the same. 

<ins>**NOTE**</ins>: Since the array does not include time information, the duration for the whole process is considered to be 1, hence it simplifies a lot of the calculations. We call this "normalized time". If you need the time information included you should refer to the main references. 

### Parabolic Method

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


</br>
<div align="center">
 	<img src="https://github.com/user-attachments/assets/e3d2bb5a-3c3e-4f1a-b4cd-2a5ebadfcd7d" style="width: 50%;">
	</br>
	Bang-Bang Method | Parabolic Method
</div>
</br>

### Trapezoidal Method

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


So we can calculate all of the $\theta, \dot{\theta}, \ddot{\theta}$. First let's go for the easiest one, $\ddot{\theta}$:


```math
\begin{aligned}
	\ddot{\theta}(t) & = a & \quad \text{for} \quad 0 \leq t \leq T \\
	\ddot{\theta}(t) & = 0 & \quad \text{for} \quad T \leq t \leq 2T \\
	\ddot{\theta}(t) & = -a & \quad \text{for} \quad 2T \leq t \leq 1 \\
\end{aligned}
```

Next up is $\dot{\theta}$: 

```math
\begin{aligned}
	\dot{\theta}(t) & = a t & \quad \text{for} \quad 0 \leq t \leq T \\
	\dot{\theta}(t) & = v_{\text{max}} & \quad \text{for} \quad T \leq t \leq 2T \\
	\dot{\theta}(t) & = -a(t - 2T) + v_{\text{max}} & \quad \text{for} \quad 2T \leq t \leq 1 \\
\end{aligned}
```

And the next is $\theta$:

```math
\begin{aligned}
	\theta(t) & = \theta_0 + \frac{1}{2} a t^2 & \quad \text{for} \quad 0 \leq t \leq T \\
	\theta(t) & = \theta_0 + \frac{1}{2} a T^2 + v_{\text{max}} (t - T) & \quad \text{for} \quad T \leq t \leq 2T \\
	\theta(t) & = \theta_0 + \frac{1}{2} a T^2 + v_{\text{max}} T + v_{\text{max}} (t - 2T) - \frac{1}{2} a (t - 2T)^2 & \quad \text{for} \quad 2T \leq t \leq 1 \\
\end{aligned}
```

where 
- $T = \frac{1}{3}$
- $v_{\text{max}} = \frac{\theta_f - \theta_i}{1 - T}$
- $a = 3 v_{\text{max}}$

</br>
<div align="center">
 	<img src="https://github.com/user-attachments/assets/580f77ad-6dfa-4b17-b2b8-07fa0739bb91" style="width: 50%;">
	</br>
	Trapezoidal Method
</div>
</br>


### S-curve Method

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

We'll do the same thing as the trapezoidal method and calculate the entire movement based on $\theta, \dot{\theta}, \ddot{\theta}, \dddot{\theta}$. First is $\dddot{\theta}$: 

```math
\begin{aligned}
	\dddot{\theta}(t) & = j_{\text{max}} & \quad \text{for} \quad 0 \leq t \leq T \\
	\dddot{\theta}(t) & = 0 & \quad \text{for} \quad T \leq t \leq 2T \\
	\dddot{\theta}(t) & = - j_{\text{max}} & \quad \text{for} \quad 2T \leq t \leq 3T \\
	\dddot{\theta}(t) & = 0 & \quad \text{for} \quad 3T \leq t \leq 4T \\
	\dddot{\theta}(t) & = - j_{\text{max}} & \quad \text{for} \quad 4T \leq t \leq 5T \\
	\dddot{\theta}(t) & = 0 & \quad \text{for} \quad 5T \leq t \leq 6T \\
	\dddot{\theta}(t) & = j_{\text{max}} & \quad \text{for} \quad 6T \leq t \leq 7T \\
\end{aligned}
```

Next is $\ddot{\theta}$: 

```math
\begin{aligned}
	\ddot{\theta}(t) & = j_{\text{max}} t & \quad \text{for} \quad 0 \leq t \leq T \\
	\ddot{\theta}(t) & = a_{\text{max}} & \quad \text{for} \quad T \leq t \leq 2T \\
	\ddot{\theta}(t) & = a_{\text{max}} - j_{\text{max}} (t - 2T) & \quad \text{for} \quad 2T \leq t \leq 3T \\
	\ddot{\theta}(t) & = 0 & \quad \text{for} \quad 3T \leq t \leq 4T \\
	\ddot{\theta}(t) & = - j_{\text{max}} (t - 4T) & \quad \text{for} \quad 4T \leq t \leq 5T \\
	\ddot{\theta}(t) & = - a_{\text{max}} & \quad \text{for} \quad 5T \leq t \leq 6T \\
	\ddot{\theta}(t) & = - a_{\text{max}} + j_{\text{max}} (t - 6T) & \quad \text{for} \quad 6T \leq t \leq 7T \\
\end{aligned}
```

Next is $\dot{\theta}$: 


```math
\begin{aligned}
	\dot{\theta}(t) & = \frac{1}{2} j_{\text{max}} t^2 & \quad \text{Phase} 1\\
	\dot{\theta}(t) & = \frac{1}{2} a_{\text{max}} T + a_{\text{max}} (t - T) & \quad \text{Phase} 2 \\
	\dot{\theta}(t) & = \frac{3}{2} a_{\text{max}} T + a_{\text{max}} (t - 2T) - \frac{1}{2} j_{\text{max}} (t - 2T)^2 & \quad \text{Phase} 3\\
	\dot{\theta}(t) & = v_{\text{max}} &  \quad \text{Phase} 4\\
	\dot{\theta}(t) & = v_{\text{max}} - \frac{1}{2} j_{\text{max}} (t - 4T)^2 &  \quad \text{Phase} 5 \\
	\dot{\theta}(t) & = \frac{3}{4} v_{\text{max}} - a_{\text{max}} (t - 5T) &  \quad \text{Phase} 6\\
	\dot{\theta}(t) & = \frac{1}{4} v_{\text{max}} - a_{\text{max}} (t - 6T) + \frac{1}{2} j_{\text{max}} (t - 6T)^2 &  \quad \text{Phase} 7 \\
\end{aligned}
```

Finally it's $\theta$: 


```math
\begin{aligned}
	\theta(t) & = \theta_0 + \frac{1}{6} j_{\text{max}} t^3 & \quad \text{Phase} 1\\
	\theta(t) & = \theta_0 + \frac{1}{12} v_{\text{max}} T + \frac{1}{2} a_{\text{max}} T(t - T) + \frac{1}{2} a_{\text{max}} (t - T)^2 &  \quad \text{Phase} 2\\
	\theta(t) & = \theta_0 + \frac{7}{12} v_{\text{max}} T + \frac{3}{2} a_{\text{max}} T(t - 2T) + \frac{1}{2} a_{\text{max}} (t - 2T)^2 - \frac{1}{6} j_{\text{max}} (t - 2T)^3 &  \quad \text{Phase} 3\\
	\theta(t) & = \theta_0 + \frac{3}{2} v_{\text{max}} T + v_{\text{max}} (t - 3T) &  \quad \text{Phase} 4 \\
	\theta(t) & = \theta_0 + \frac{5}{2} v_{\text{max}} T + v_{\text{max}} (t - 4T) - \frac{1}{6} j_{\text{max}} (t - 4T)^3 & \quad \text{Phase} 5\\
	\theta(t) & = \theta_0 + \frac{41}{12} v_{\text{max}} T + \frac{3}{4} v_{\text{max}} (t - 5T) - \frac{1}{2} a_{\text{max}} (t - 5T)^2 &  \quad \text{Phase} 6\\
	\theta(t) & = \theta_0 + \frac{47}{12} v_{\text{max}} T + \frac{1}{4} v_{\text{max}} (t - 6T) - \frac{1}{2} a_{\text{max}} (t - 6T)^2 + \frac{1}{6} j_{\text{max}} (t - 6T)^3 & \quad \text{Phase} 7 \\
\end{aligned}
```

Where:
- $T = \frac{1}{7}$
- $v_{\text{max}} = \frac{\theta_f - \theta_i}{4T}$
- $a_{\text{max}} = \frac{v_{\text{max}}}{2T}$
- $j_{\text{max}} = \frac{a_{\text{max}}}{T}$


</br>
<div align="center">
 	<img src="https://github.com/user-attachments/assets/84d06621-bce1-4361-840f-71a673953985" style="width: 50%;">
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
 	<img src="https://github.com/user-attachments/assets/34f6998f-b737-44d6-90f1-c018b7d0799b" style="width: 50%;">
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
 	<img src="https://github.com/user-attachments/assets/b79e04c0-35db-4c2c-8780-44855ce16d02" style="width: 50%;">
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
 	<img src="https://github.com/user-attachments/assets/b86a731d-fb5a-453a-b687-7651932c5570" style="width: 50%;">
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

</br>
<div align="center">
 	<img src="https://github.com/user-attachments/assets/deeacfa3-7191-4cb6-983b-3795b64b6b93" style="width: 50%;">
	</br>
	Multi-point movement
</div>
</br>

Multi-point trajectory generation generates a trajectory between multiple target points given as a path, along with its time information. For example if the robot needs to be moved from point A, passes point B and then stop at point C (same logic with more points, basically we have a path which contains more than the starting and final points).

So like the point-to-point methods the **inputs** are the path points which want to hit, and then the **output** is the array of values (sampled from the position as a function of time) that we'll give to the PID Controller as a reference. 

### Point-to-Point Methods 

**How can we use a point-to-point method for multiple points?** It’s straightforward. Let's revisit the previous example, where we want to create a trajectory for the values [0, 0.2, 1]. First, we can generate a trajectory between [0, 0.2], and then create another trajectory between [0.2, 1]. This approach works for any number of points and can be applied using any of the six point-to-point methods mentioned. **However, this method might not be suitable for certain tasks.** For instance, in applications where smooth, continuous motion is required, we may not want to stop at each path point. In such cases, using these point-to-point methods may not be the best idea. Here's an example of where this method might be applied:

</br>
<div align="center">
 	<img src="https://github.com/user-attachments/assets/02cdf85e-d61f-4f3b-afe9-3974e3a63551" style="width: 50%;">
	</br>
	In this application of petri dish sampling in microorganism culturing (which my team implemented), the robot is required to stop at each path point because a liquid drop needs to be placed at each location.
</div>
</br>

And the overall idea is to interpolate between each two points in the path, with a point-to-point method. If we interpolate between four values of [0, 1, -1, 0], the following diagrams are what we're getting:

### High-Order Polynomial Interpolation

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

### Cubic Spline 

This method basically uses multiple polynomials on end for generating a trajectory between multiple path points. So if we are given $n+1$ path points, we'll need $n$ polynomials to interpolate and create the trajectory. 

Each polynomial will have a degree of $p$ based on the desired level of trajectory smoothness. As default we'll have $p=3$ which will give us a smooth velocity profile and a continuous acceleration profile

So the input is:

```math
\begin{aligned}
	\text{path array} = [\theta_0, \theta_1, \dots, \theta_n]   
\end{aligned}
```

The overall trajectory function can be described as: 

```math
\begin{aligned}
	\theta(t) & = \lbrace q_k(t), t \in [t_k, t_{k+1}], k=0, \dots , n-1 \rbrace \\
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
	& q_k(t_k) = \theta_k, \quad q_k(t_{k+1}) = \theta_{k+1}, & k=0, ..., n-1 \\
	& \dot{q}_k (t_{k+1}) 	= \dot{q}_{k+1}(t_{k+1}), & k=0, ..., n-2\\
	& \ddot{q}_k (t_{k+1}) 	= \ddot{q}_{k+1} (t_{k+1}), & k=0, ..., n-2\\
	& \dot{q}_0 (t_0) 	= v_0 = 0 , \quad \dot{q}_{n-1} (t_n) = v_n = 0 & \\
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

```math
\begin{cases}
    a_{k,0} & = q_k\\ 
    a_{k,1} & = v_k\\ 
    a_{k,2} & = \frac{1}{T_k}   [\frac{3(q_{k+1} - q_k)}{T_k} - 2v_k - v_{k+1}] \\ 
    a_{k,3} & = \frac{1}{T^2_k} [\frac{2(q_k - q_{k+1})}{T_k} + v_k + v_{k+1}] 
\end{cases}
```


But this is for when the velocities of the points are known, which they are not (except the initial and final points). So the velocities have to be calculated, in this instance we use the continuity conditions of acceleration. Velocities can be found with a matrix of $v = A^{-1}c$. Where: 


$$
A = 
\begin{bmatrix}
    2(T_0+T_1)     & T_0         & 0       & ...                             &     & 0 \\
    T_2            & 2(T_1+T_2)  & T_1     & 0                               &     & \vdots \\
    0              &             & \ddots  &                                 &     & 0 \\
    \vdots         &             &         & T_{n-2}  & 2(T_{n-3}+T_{n-2})   & T_{n-3} \\ 
    0              & \dots       &         & 0        & T_{n-1}              & 2(T_{n-2} + T_{n-1}) \\  
\end{bmatrix}
$$

$$
c = 
\begin{bmatrix}
    \frac{3}{T_0T_1} \left[ T^2_0(q_2 - q_1) + T^2_1(q_1 - q_0) \right] - T_1 v_0 \\
    \frac{3}{T_1T_2} \left[ T^2_1(q_3 - q_2) + T^2_2(q_2 - q_1) \right] \\
    \vdots \\ 
    \frac{3}{T_{n-3}T_{n-2}} \left[ T^2_{n-3}(q_{n-1} - q_{n-2}) + T^2_{n-2}(q_{n-2} - q_{n-3}) \right] \\
    \frac{3}{T_{n-2}T_{n-1}} \left[ T^2_{n-2}(q_{n} - q_{n-1}) + T^2_{n-1}(q_{n-1} - q_{n-2}) \right] - T_{n-2}v_{n} \\
\end{bmatrix}
$$

$$
v = 
\begin{bmatrix}
    v_1 \\ 
    v_2 \\ 
    \vdots \\ 
    v_{n-2} \\ 
    v_{n-1} \\ 
\end{bmatrix}
$$

For better understanding please refer to [3] in the reference section. 

</br>
<div align="center">
 	<img src="https://github.com/user-attachments/assets/0a68a360-92fd-484d-ab30-4535dd30bc71" style="width: 50%;">
	</br>
	Cubic Spline Method
</div>
</br>


### Pattern Generation 

This one is a bit tough to explain so bear with me for a second. When you have a certain pattern the previous trajectory planning methods won't work. Why is that? Because in the previous methods we were given a few number of points and we wanted to generate a trajectory in-between those points, but with a 'pattern' we already have the trajectory that the robot needs to go.

So in this instance what we should do is to sample that pattern which results in an array of values much like before, which we will give to the PID controller of the robot as a reference to follow. How do we sample the pattern? well what I do is this: I first sample at a constant rate (based on distance) then I'll repeat the values which correspond to the sharp edges of the pattern. This way I can emphasis on the sharp ends for the PID controller so it comes out sharper (because the PID controller will have more time to reach it's destination with more precision). That's it. 


<div align="center">
 	<img src="https://github.com/user-attachments/assets/2dda7d86-4144-4927-a77d-dd997b848776" style="width: 50%;">
 	<img src="https://github.com/user-attachments/assets/5ed8a76f-bad9-44ca-99b6-00f791b0570e" style="width: 50%;">
	</br>
	Pattern Generation 
</div>
</br>


### Discussion 

So we took a look at a few multi-point methods:

1. Higher-Order Polynomials for Multiple Points
2. Trapezoidal / Cubic Polynomial / Quintic Polynomial Multi-Point (Same as Point-to-Point Methods)
3. Cubic Spline
4. B-Spline
5. Pattern Generation

So let's talk about why you should or shouldn't use each of these algorithms:

* **Higher-order interpolating polynomial:** I can say that it works ... but it's very impracticaly. _The reason being that the calculations can not be generalized._
* **Point-to-point methods:** being used for multiple points (Trapezoidal / Cubic Polynomial / Quintic Polynomial Multi-Point), that's just plain stupid. _The reason for that is the velocity comes down to zero at every main time instance._
*  **Cubic Spline:** Well in my opinion this is the best method if you have a few _path points that you want to absolutely hit_. it just works well. Of course there are ways to modify it but we're not gonna go into that hell hole. just remember that you can add extra conditions for a higher order polynomial used in this. But of course you have to do the calculations from the top.
*  **B-Spline:** In terms of smoothness this method is really good (better than the cubic spline probably) but the thing is ... _it doesn't hit the path points EXACTLY_. it's basically an approximation. So honestly if my goal was low error for the control points I've defined ... I wouldn't use this one.
*  **Pattern Generation**: This one is just a bit messy to be honest and I don't know how to improve on it. It's got something to do with sampling rate or whatever ... but putting that aside _when you basically have a certain pattern that you want your robot to travel_ you can just make an array of the actuator parameter values based on that shape. Then add a couple of extra emphasis points on the sharp edges manually, and then you'll have a trajectory that works for your pattern. 


<a name="section-adeptcycle"></a>
## Adept Cycle 

Well what is adept cycle? it's basically just four points in 3D space that the robot must hit in order to perform a pick-and-place operation. it looks something like this:


<div align="center">
 	<img src="https://github.com/user-attachments/assets/78efffa4-90ac-4bd5-b245-08d04c101c30" style="width: 50%;">
	</br>
	Adept Cycle  
</div>
</br>


Amazing. Now that we know what adept cycle looks like, I should say that there a number of ways to interpolate those 4 points (as we've been discussing). So here are the methods used and their respective plots 


<div align="center">
 	<img src="https://github.com/user-attachments/assets/8fd13a69-5cc1-49db-bb67-685fe30f8f97" style="width: 50%;">
 	<img src="https://github.com/user-attachments/assets/7d3fd81f-3411-4932-8bcc-647d75cc21fc" style="width: 50%;">
	</br>
	Point-to-Point Parabolic Method
</div>
</br>


<div align="center">
 	<img src="https://github.com/user-attachments/assets/dcb508f5-3853-442b-987b-89395aed73c0" style="width: 50%;">
 	<img src="https://github.com/user-attachments/assets/4f0011a4-fd8c-4174-859c-905a8c117355" style="width: 50%;">
	</br>
	Point-to-Point Trapezoidal Method
</div>
</br>

<div align="center">
 	<img src="https://github.com/user-attachments/assets/7c8a2b80-ebf3-450e-871c-48f83f1b032d" style="width: 50%;">
 	<img src="https://github.com/user-attachments/assets/aa727c05-0f36-4478-a7e9-d44627821bdd" style="width: 50%;">
	</br>
	Point-to-Point S-Curve Method	
</div>
</br>

<div align="center">
 	<img src="https://github.com/user-attachments/assets/fe54642f-bd87-42b9-bac7-7b3bf7dd3e3b" style="width: 50%;">
 	<img src="https://github.com/user-attachments/assets/54666eba-44ee-4a94-b5a3-c70d63658ee9" style="width: 50%;">
	</br>
	Point-to-Point 5th Order Interpolating Polynomial	
</div>
</br>


<div align="center">
 	<img src="https://github.com/user-attachments/assets/af8cb949-8093-4128-a8f9-72b66ab964f9" style="width: 50%;">
 	<img src="https://github.com/user-attachments/assets/5c3a91cf-6f68-427b-be2b-eb53b72609cf" style="width: 50%;">
	</br>
	Point-to-Point 7th Order Interpolating Polynomial	
</div>
</br>


<div align="center">
 	<img src="https://github.com/user-attachments/assets/68d1caa8-e3cd-4975-8978-057cb443a89e" style="width: 50%;">
 	<img src="https://github.com/user-attachments/assets/641224e3-1679-46b2-9f79-c59d6bfece0f" style="width: 50%;">
	</br>
	Point-to-Point 9th Order Interpolating Polynomial	
</div>
</br>


<div align="center">
 	<img src="https://github.com/user-attachments/assets/119ac36d-08ee-45b4-ab0a-10b40b777f73" style="width: 50%;">
 	<img src="https://github.com/user-attachments/assets/7b4f744d-d373-45de-a14b-6197ef19a5b8" style="width: 50%;">
	</br>
	Cubic Spline
</div>
</br>

<div align="center">
 	<img src="https://github.com/user-attachments/assets/72c8f54a-89d5-4a74-b17a-32f19b526d09" style="width: 50%;">
 	<img src="https://github.com/user-attachments/assets/eecc827f-8996-4ea2-af92-246f70a614fb" style="width: 50%;">
	</br>
	Cubic Spline with modified Adept Cycle Points (to prevent the overshooting)
</div>
</br>

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


### PathPlannerMLTP.py

This file introduces a multi-point path planner class and for initiating the class you can write:

```python
# Set the path 
PATH = [0, -0.2, 0.3, 0.8, -0.1, 1]

# initialize the path planner class 
path_planner = PathPlannerMLTP(PATH)

```

Here are the multi-point trajectory planning methods implemented: 

1. Cubic Spline
2. B-Spline

```python
# calculate the trajectory based on cubic spline 
results = path_planner.mltp_cubicspline()
path_planner.plot(results, "cubic spline")

# calculate the trajectory based on B spline 
results = path_planner.mltp_bspline()
path_planner.plot(results, "B spline")
```

### PathPlannerAdeptCycle.py

Here's an example of the adept cycle usage. figure it out yourself 

```python
# Set the path [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]]
PATH = [[0, 0, 0], [0.15, 0.15, 0.5], [0.85, 0.85, 0.5], [1, 1, 0]]

# Path planner class init
path_planner = PathPlanner_AdeptCycle(PATH)

# getting the results for the cubic spline method
(xyz_array, xyz_results) = path_planner.cubic_spline()

# path planner plot 3D 
path_planner.plot3d(xyz_array, 'Cubic Spline')
path_planner.plot(xyz_results, 'Cubic Spline')
```

The methods used for trajectory generation are ```cubic_spline()``` and ```point_to_point(interpolation_method)``` where the interpolation method can be the following values:

* ```interpolation_method = 'ptp_polynomial5th'```
* ```interpolation_method = 'ptp_polynomial7th'```
* ```interpolation_method = 'ptp_polynomial9th'```
* ```interpolation_method = 'ptp_bangbang'```
* ```interpolation_method = 'ptp_trapezoidal'```
* ```interpolation_method = 'ptp_scurve'```


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

