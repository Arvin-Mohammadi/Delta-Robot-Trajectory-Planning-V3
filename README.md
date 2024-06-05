# Delta Parallel Robot - Kinematics & Trajectory Planning - Theoretical & Experimental Study
*sigh* ok let's do this one last time. 


Overview: 
- [Introduction](#section-introduction)
- [Delta Robot Kinematics](section-deltarobot_kinematics)
- [Theoretical Study - Point-to-Point Trajectory Generation](section-point2point_trajectory_generation)
- [Theoretical Study - Multi-Point Trajectory Generation](section-multipoint_trajectory_generation)
- [References](section-references)
- [End Notes](section-endnote)

</br>

![DSC03194_edited-min](https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/5d0b34e0-8cbd-4d3d-9884-382a565008ef)


<a name="section-introduction"></a>
## Introduction
------

<ins>**What is a Delta Robot and Why is it Important**</ins> 

People who search for this repository already know what the delta robot is and know why it's important. Delta robot parallel kinematic structure and high-speed capabilities make them ideal for precise and speedy tasks, particularly in pick-and-place operations. This repository studies trajectory planning methods for Delta robots, focusing on smooth motion for the End-Effector while minimizing deviations.

**The applications of Delta robots** are virtually endless but mainly they come down to a certain pattern of pick-and-place or 3D printing or tasks that require somewhat similar movements to these two. Here's three applications that I worked on (PLEASE CITE MY 3 PAPERS):

- Pick-and-Place Operations: [LINK](https://ieeexplore.ieee.org/abstract/document/10334699)
- Automated Pipetting Operation: [LINK](https://ieeexplore.ieee.org/abstract/document/10412424)
- Food Packaging - Not Published Yet
- Automated Pipetting Operation: Not Published Yet

<ins>**Pros and Cons of Delta Robot**</ins>

Delta robot has basically two advantages at the cost for two things (I'm talking about the important ones). 

Advantages: 
1. High Speed
2. High Precision

Disadvantages:
1. Small Workspace
2. Low Workload

<ins>**Trajectory Planning**</ins> 

Take a look at the figure below. There are four main stages to any sort of robotic operation:
1. **Task Planning:** Figuring out what the robot is going to be doing [in order]
2. **Path Planning:** What points in 3D space the robot is going through [in order]
3. **Trajectory Planning:** Positon as a function of time
4. **Control**: Giving the trajectory as a reference to the robot's controller

![trajectory planning model](https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/5924887e-6c97-4af0-b3d5-d6d9a3c5c459)


<a name="section-deltarobot_kinematics"></a>
## Theoretical Study - Delta Robot Kinematics
------

<ins>**What are Forward and Inverse Kinematics**</ins> 

Let's say our robot has actuated joints of [$\theta_1, \theta_2, \theta_3$] and the position of the end-effector is [$x, y, z$]. If we are given the actuated joint parameters and want to calculate the position of end-effecotr, that's Forward Kinematics. If we are given the position of the end-effector and want to calculate the actuated joint parameters that's Inverse Kinematics. 

![IK and FK](https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/a06639cb-afbb-47c5-8a0d-340a20674f84)

<ins>**What is the Jacobian of a Robot**</ins> 

This basically has the same logic as the FK and IK but this time instead of converting between positions and angles, the conversion occurs between velocity of the end-effector and the velocity of the joint parameters. 

<ins>**Theoretical Solution of Forward and Inverse Kinematics in Delta Robot**</ins> 

![DPR scheme](https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/assets/69509720/b34332cf-6016-42b2-83ac-fc2824447b97)

The Delta robot consists of three main chains. Each chain starts from the base platform($O_0$), connects to the upper arm via the pin joint ($A_i$), connects to the lower arm via the universal joint ($B_i$), and finally connects to the end-effector via the universal joint ($C_i$). This results in a movement behaviour, where the end-effector moves parallel to the base platform and can move along three translational axes of ($x, y, z$) in 3D space. 

<a name="section-point2point_trajectory_generation"></a>
## Theoretical Study - Point-to-Point Trajectory Generation
------


<a name="section-multipoint_trajectory_generation"></a>
## Theoretical Study - Multi-Point Trajectory Generation
------


<a name="section-references"></a>
## References
------
R1 - [Kinematic Analysis of Delta Parallel Robot: Simulation Study - A. Eltayeb
](https://www.researchgate.net/publication/352787189_Kinematic_Analysis_of_Delta_Parallel_Robot_Simulation_Study) 
R2 - [?](https://github.com/Arvin-Mohammadi/Delta-Robot-Trajectory-Planning-V3/blob/main/References/Inverse%20Kinematics%20(Delta%20Robot).pdf)

<a name="section-endnote"></a>
## End Note
------
Ok real talk here. I'm absolutely sick of working on papers and things like that. I think they are pretentious and faily useless for computer and robotics projects such as this one. I think github projects are millions of times more valuable because they provide real insight and tend to lean on explaining how things work instead of trying to convince you why the work is really important. But I need to work on publishing papers and also finishing my bachelor thesis ... at least for now. But just know that sharing this information in the raw format and purely practical way like this with the people around the world who might need it, is what makes me even slightly excited about doing this. that's what keeps me going. so thanks for taking interest in the previous projects I did on Delta robot trajectory planning. this will be the final version and I will hopefully never work or even see another delta robot in my entire life also follow me on instagram if you're into gaming 🖥️🎮🤖  

