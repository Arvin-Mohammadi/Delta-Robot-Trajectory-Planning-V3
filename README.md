# Delta Parallel Robot - Kinematics, Trajectory Planning, Obstacle Avoidance, and Experimental Study 

Overview: 
- [Introduction](#section-introduction)
- [Delta Robot Kinematics](#section-deltarobot_kinematics)
  - [Theoretical Solution of Forward and Inverse Kinematics in Delta Robot](#subsection-theoretical-solution-fkik-dpr)
  - [Experimental Solution of Forward and Inverse Kinematics in Delta Robot](#subsection-experimental-solution-fkik-dpr)
- [Theoretical Study - Point-to-Point Trajectory Generation](#section-point2point_trajectory_generation)
- [Theoretical Study - Multi-Point Trajectory Generation](#section-multipoint_trajectory_generation)
- [Adept Cycle](#section-adeptcycle) 
- [References](#section-references)
- [Ending Note](#section-endnote)

<br>

<div style="float: right; width: 25%; margin-left: 10px; margin-bottom: 10px;">
  <img src="https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/5d0b34e0-8cbd-4d3d-9884-382a565008ef" style="width: 25%;">
</div>

</br>

<div align="center">
  <img src="https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/5d0b34e0-8cbd-4d3d-9884-382a565008ef" style="width: 25%;">
</div>


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
3. **Trajectory Planning:** Positon as a function of time
4. **Control**: Giving the trajectory as a reference to the robot's controller

![trajectory planning model](https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/5924887e-6c97-4af0-b3d5-d6d9a3c5c459)


<a name="section-deltarobot_kinematics"></a>
## Delta Robot Kinematics

<ins>**What are Forward and Inverse Kinematics**</ins> 

Look at the figure below. Let's say our robot has actuated joints of $\[\theta_1, \theta_2, \theta_3\]$ and the position of the end-effector is $\[x, y, z\]$:

- **Forward Kinematics:** Given the actuated joint parameters to calculate the position of end-effector
- **Inverse Kinematics:** Given the position of the end-effector to calculate the actuated joint parameters

![IK and FK](https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/a06639cb-afbb-47c5-8a0d-340a20674f84)

<ins>**What is the Jacobian of a Robot**</ins> 

This basically has the same logic as the FK and IK but this time instead of converting between positions and angles, the conversion occurs between velocity of the end-effector and the velocity of the joint parameters. 


<a name="subsection-theoretical-solution-fkik-dpr"></a>
### Theoretical Solution of Forward and Inverse Kinematics in Delta Robot

![DPR scheme](https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/b34332cf-6016-42b2-83ac-fc2824447b97)
_Note: The solution is from the reference #1_

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




<a name="subsection-experimental-solution-fkik-dpr"></a>
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

NOTE: The output array does not include any time information, the time information comes into play when we want to give the PID controller the next target point from the array. So basically when the robot wants to move from Point A to  B in either 2 seconds or 3 seconds, the interpolation array for both of these actions are the same. 

NOTE: Since the array does not include time information, the duration for the whole process is considered to be 1, hence it simplifies a lot of the calculations. We call this "normalized time". If you need the time information included you should refer to the main references. 


#### Bang-Bang 

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
\begin{algined}
	\theta_a(t) & = a_0 + a_1(t - t_0) + a_2(t - t_0)^2 & \quad\text{for}‌\quad t_0 \leq t \leq t_1 \\ 
	\theta_b(t) & = a_3 + a_4(t - t_1) + a_5(t - t_1)^2 & \quad\text{for}‌\quad t_1 \leq t \leq t_2 \\ 
\end{algined}
```



#### Trapezoidal 

As explained, the goal here is to basically use a trapezoidal diagram as a way to interpolate between two given motor rotations. The trapezoidal diagram is defined as the following:

```math
  \dot{\theta} = 
  \begin{cases}
    at        & t_0 \leq t < t_1 \\
    V_{max}   & t_1 \leq t < t_2  \\
    -at       & t_2 \leq t \leq t_3
  \end{cases}
```
For the sake of simplicity we say that $t_0 = 0, t_1 = T/3, t_2 = 2T/3, t_3 = T$. We have from the given problem that $T=1s$, which results in:

```math
  \dot{\theta} = 
  \begin{cases}
    at        & 0 \leq t < 1/3 \\
    V_{max}   & 1/3 \leq t < 2/3  \\
    -at       & 2/3 \leq t \leq 1
  \end{cases}
```

We know that $v_{max} = a.t_1 = a/3$ so we can calculate the acceleration given that the robot has a limitation of top-speed.  


#### S-curve

The S-curve method is similar to the trapezoidal method with the difference that it is smoother. So in mathemtical terms that would be: 

```math
  \ddot{\theta} = 
  \begin{cases}
    jt        & t_0 \leq t < t_1 \\
    a_{max}   & t_1 \leq t < t_2  \\
    -jt       & t_2 \leq t < t_3 \\
    -a_{max}  & t_3 \leq t < t_4 \\
    jt        & t_4 \leq t \leq t_5
  \end{cases}
```

#### 5th order polynomial 

#### 7th order polynomial 

#### higher order polynomial

<a name="subsection-multipoint_trajectory_generation"></a>
### Theoretical Study - Multi-Point Trajectory Generation

Multi-point trajectory generation generates a trajectory between multiple target points given as a path, along with its time information. For example if the robot needs to be moved from point A, passes point B and then stop at point C.  


#### higher order polynomial 


#### trapezoidal / cubic polynomial / quintic polynomial


#### Multi-Segment Linear Trajectory with Trapezoidal Blends


#### Multi-Segment Linear Trajectory with Polynomial Blends


#### Cubic Spline 


#### B-Spline


#### Pattern Generation 


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

