---
layout: default
title: (1) Simulating a multi-link soft manipulator (PCC)
parent: Modeling and Control 
grand_parent: Examples
nav_order: 1
permalink: /docs/examples/model_control/multilink/
---

#  Simulating the dynamics of a multi-link soft robot manipulator (PCC approach)
{: .no_toc }

<details open markdown="block">
  <summary>
    Table of contents
  </summary>
  {: .text-delta }
1. TOC
{:toc}
</details>
---

#### Difficulty: `intermediate`{: .fs-3 .text-green-200}
{: .no_toc }
 - Required classes: `Shapes.m`{: .text-purple-000}, `Model.m`{: .text-purple-000}, `Rig.m`{: .text-purple-000}
 - Code length: `~25 lines`{: .text-purple-000} (without comments)

---

### Introduction
In the following example, we will build a multi-link soft robot manipulator, similar to Festo's Bionic Handling Assistant as shown below. The system consists of three pneumatic segments, which we will model according the *Piecewise-Constant-Curvature* description -- PCC in short. SOROTOKI comes equipped with two Classes: `Shapes.m`{: .text-purple-000} and `Model.m`{: .text-purple-000} which take care of the spatial strain parameterization and the continuum dynamic model, respectively. Furthermore, `Shapes.m`{: .text-purple-000} will also contain the physical properties of the spatial beam, such as density, stiffness, and damping coefficients. Once the dynamic trajectories of the system are solved using `Model.m`{: .text-purple-000}, we can call `Rig.m`{: .text-purple-000} to assign an kinematic rig for 3D-graphical models. The resulting simulations is produced using SOROTOKI:

<div align="center">
<img src="./img/intro.jpg" width="500"> </div>
<div align="center">
Festo's Bionic Handling Assistant inspired by the elphant's trunk (see [1])  
</div>

<div align="center">
<img src="./img/mdl_threelink.gif" width="500"> </div>
<div align="center">
Three-link soft robot manipulator dynamics using SOROTOKI based on the Festo's Bionic Handling Assistant. The model's state dimension is $\dim(q) = 6$ that is two unique $x-z$ and $y-z$ curvatures per soft-link segment. 
</div>


### Strain parameterization
To describe the spatial evolution of the soft robot, we use the class `Shapes.m`. The input for the class is a matrix $Y$ of size $N \times M$, where $N$ is the number of spatial discretizations (i.e., the number of nodes along the spatial curve) and $M$ the number of unique modes. Note that the columns of $Y$ correspond to the spatial modes of the soft robotic model.

```matlab
%% spatial parameterization
L = 360;  % intrinsic length of soft robot
N = 300;  % number of spatial nodes
M = 3;    % number of modes (i.e., discrete links)

% construct spatial parameters X
X = linspace(0,L,N)';
Y = [];

% constructing the shape function matrix evaluated for X
for ii = 1:M
   Y(:,ii) = pcc(X/L,ii,M); % PCC 
end

shp = Shapes(Y,Modes,'L0',L);
shp.show();

```

The code above should produce the following:

<div align="center"> <img src="./img/shapes.png" width="550"> </div> 
<div align="center"> Shape functions of the three-link soft robot </div>


[**[1]**](https://www.festo.com/group/en/cms/10241.htm) **Bionic Handling Assistant** a soft robotic manipulator from Festo.
{: .fs-3} 

