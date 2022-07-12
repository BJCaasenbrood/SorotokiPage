---
layout: default
title: (1) Simulating a multi-link soft manipulator (PCC)
parent: Modeling and Control 
grand_parent: Examples
nav_order: 1
permalink: /docs/examples/model_control/multilink/
---

<script>
MathJax = {
  tex: {
    inlineMath: [['$', '$'], ['\\(', '\\)']]
  },
  svg: {
    fontCache: 'global'
  }
};
</script>
<script type="text/javascript" id="MathJax-script" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js">
</script>

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
To describe the spatial evolution of the soft robot, we use the class `Shapes.m`{: .text-purple-000}. The input for the class is a matrix $Y$ of size $N \times M$, where $N$ is the number of spatial discretizations (i.e., the number of nodes along the spatial curve) and $M$ the number of unique modes. Note that the columns of $Y$ correspond to the spatial modes of the soft robotic model.

```matlab
%% spatial parameterization
L = 360;  % intrinsic length of soft robot
N = 300;  % number of spatial nodes

% construct spatial parameters X
X = linspace(0,L,N)';
Y = [];

% constructing the shape function matrix evaluated for X
for ii = 1:M
   Y(:,ii) = pcc(X/L,ii,M); % PCC 
end

% generate Shapes class
shp = Shapes(Y,Modes,'L0',L); % construct shape class
shp.show();                   % displays the columns of Y
```

The code above should produce the following:

<div align="center"> <img src="./img/shapes.png" width="550"> </div> 
<div align="center"> Shape functions of the three-link soft robot -- a three segmented piecewise constant strain. </div>

**Note:** By default, inputting an matrix into `Shapes`{: .text-purple-000} will ensure each column is mutually orthogonal using the Gram-Schmidt orthogonalization procedure, see [[2]](https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process). In other words, the integral $\int_0^L Y_i^\top Y_j \; d\sigma $ will be $1$ for indices $i = j$, and $0$ otherwise. Naturally, this ensure the spatial modes are decoupled from a compliance perspective.

**Note:** Second, notice we specify `Shapes(...,'L0',L)`{: .text-purple-000}. This ensure the intrinsic length of the curve is equal to $L$. By default, the intrinsic length is set to $L = 1$ if not adjusted by the user.

## Material assignment
Analogous to the finite element class, we can assign materials using `Shapes.Material`{: .text-purple-000}. Here, let us consider a Neo-Hookean material with Young's modulus $E = 10$ MPa and poisson ratio $\nu = 0.4$. We also change the density to $\rho = 150 $ kg/m$^3$ which translates to `Shapes.Material.Rho = 150e-12`{: .text-purple-000} as SOROTOKI uses unit millimeters (mm). After those assignments, we set the gravitational component using `Shapes.Gvec`{: .text-purple-000}. Once all physical properties are set, make sure to rebuild the necessary material tensors using `shp.rebuild()`{: .text-purple-000}.

```matlab
%% setting Neo-Hookean material
shp.Material = NeoHookeanMaterial(10,0.4);

% overwriting properties
shp.Material.Rho = 150e-12;    % setting density
shp.Gravity = [-9.81e3; 0; 0]; % setting gravity

% rebuild material tensors
shp = shp.rebuild();
```

## Constructing the Model class
Once the shapes and the physical parameters of the curve are set, we can construct the model class. To do so, we simply input the `shp`{: .text-purple-000} into `Model.m`{: .text-purple-000}. We also have to specify which Degrees-of-freedom we want to have enabled for the curve. Consider the vector `Modes = [kxx,kxz,kyz,exx,exz,eyz]`{: .text-purple-000} where `[kxx,kxz,kyz]`{: .text-purple-000} are the twist and curvatures modes, and `[exx,exz,eyz]`{: .text-purple-000} the elongation and shear modes. Since we consider a system undergoing pure bending specified by three PCC functions, we have `Modes = [0,kxz,kyz,0,0,0]`{: .text-purple-000} with `kxz = kyz = 3`{: .text-purple-000}.

```matlab
% generating Model class
Modes = [0,3,3,0,0,0]   % DOFs of the curve, i.e., only pure bending
mdl = Model(shp,Modes); % model class
```

**Important!** Note that the number of modes per DOF must corresponds to the number of columns of the matrix $Y$. let it be clear that the curvatures `kxy`{: .text-purple-000} and `kyz`{: .text-purple-000} do not have to share the same spatial description `Y`{: .text-purple-000}, as such, we can enrich the `Shapes`{: .text-purple-000} class by `shp = Shapes([Y1,Y2])`{: .text-purple-000} where `Y1`{: .text-purple-000} and `Y2`{: .text-purple-000} correspond to the spatial modes related to the curvatures `kxy`{: .text-purple-000} and `kyz`{: .text-purple-000}, respectively.

## Assigning a controller
The advantage of SOROTOKI is its own ODE solver that allows for easy controller development. To be more specific, the function which computes the generalized forces acting on the soft robot, given by `Model.tau`{: .text-purple-000}, can be overwritten by the user as to specify the torques a every timestep. For each function call, the solver will the class itself into `Model.tau`{: .text-purple-000}. Let us clarify why this is important, by considering the following example:

Suppose we want to develop an PD+ controller, that is, a Proportional-Derivative controller that compensates gravity and stiffness. Given the error $e:=q - q_d$, the PD+ controller has the following form:

$$\tau = \nabla_{q}\;\mathcal{U}(q) - K_p e - K_d \dot{e} $$

where we choose $K_p = k_1 K_T$ with tangent stiffness $K_T = \nabla^2_{q}\;\mathcal{U}_e(q)$, and $K_d = k_2 M$ with $M(q)$ the generalized inertia matrix; and $k_1,k_2 > 0$ control gains. For simplicity, lets consider the following controller gains: $k_1 = k_2 = 1$. To construct the controller in SOROTOKI, consider the following:

```matlab
% desired joint configuration
qd = zeros(shp.NDim,2);
qd(1,1) = +5e-3; % curvature in 1/mm  
qd(1,2) = -5e-3; % curvature in 1/mm   
qd(5,:) = -7e-3; % curvature in 1/mm  
qd(6,:) = -7e-3; % curvature in 1/mm  

% assign controller
mdl.tau = @(m) Controller(m,qd)

% auxiliary function for controller
function tau = Controller(mdl,Qd)
  q  = mdl.Log.q;   % get q(t)
  dq = mdl.Log.dq;  % get dq(t)

  Kp  = mdl.Log.EL.K;     % tangent stiffness matrix
  Kd  = KT*mdl.Log.EL.M;  % inertia matrix

  % switching set-point
  if mdl.Log.t < 5
    e = q - Qd(:,1);
  else
    e = q - Qd(:,2);
  end

  % PD+ control law
  tau = mdl.Log.dUdq - Kp*e - Kd*dq;
end
```

## Simulating the closed-loop system
Now, we only have to simulate the system. Let us consider a finite horizon time of $T = 10$ s with timesteps $\Delta t = 10$ ms. Then, we can simulate the closed-loop model as follows:

```matlab
% set solver setttings
mdl.set('TimeEnd',10,'TimeStep',10e-3);

% simulate!
mdl.simulate();

% plotting
figure(99); 
subplot(1,2,1); 
plot(mdl.Log.t,mdl.Log.q,'Linew',2); hold on;
plot([0 5 5 10],[qd(:,1),qd(:,1),qd(:,2),qd(:,2)],'k--','Linew',1);

title('states $q$','interpreter','latex',fontsize',21)
grid on; 

subplot(1,2,2); plot(mdl.Log.t,mdl.Log.p,'Linew',2);
title('momenta $p = M(q)\dot{q}$','interpreter','latex','fontsize',21);
grid on; 
```

<div align="center"> <img src="./img/states.png" width="600"> </div> 
<div align="center"> Evolution of the states and generalized momenta of the closed-loop soft robotic system. The dashed black lines denote the desired state configurations $q_d$. </div>

## Complete code 

```matlab
L = 360;   % length of robot
M = 3;     % number of modes
N = 300;   % number of discrete points on curve
H = 1/60;  % timesteps
FPS = 20;  % animation speed

Modes = [0,M,M,0,0,0];  % pure-XY curvature

%% generate shapes
X   = linspace(0,L,N)';

Y = zeros(N,M);
for ii = 1:M
   Y(:,ii) = pcc(X/L,ii,M); % pcc
end

shp = Shapes(Y,Modes,'L0',L);

%% setting materials
shp.Material = NeoHookeanMaterial(10,0.4);
shp.set('Rho',150e-12,...
        'Gravity',[-9.81e3; 0; 0]);

shp = shp.rebuild();

%% model class
mdl = Model(shp,'TimeStep',H,'TimeEnd',10);

%% class build + controller
qd = zeros(6,2);
qd(1,1) = +5*mm;
qd(1,2) = -5*mm;
qd(5,:) = -7*mm;
qd(6,:) = -7*mm;

mdl.tau  = @(M) Controller(M,qd);

%% simulate
mdl = mdl.simulate(); 

% plotting
figure(99); clf;
subplot(1,2,1); plot(mdl.Log.t,mdl.Log.q,'Linew',2); hold on;
plot([0 5 5 10],[qd(:,1),qd(:,1),qd(:,2),qd(:,2)],'k--','Linew',1);
grid on; title('states $q$','interpreter','latex','fontsize',21)
subplot(1,2,2); plot(mdl.Log.t,mdl.Log.p,'Linew',2);
grid on; title('momenta $p = M(q)\dot{q}$','interpreter','latex',...
    'fontsize',21);
sorocolor

%% animation
figure(101); clf;
rig = setupRig(M,L,Modes);

for ii = 1:fps(mdl.Log.t,FPS):length(mdl.Log.q)

    rig = rig.computeFK(mdl.Log.q(ii,:));
    rig = rig.update();
  
    axis([-.1*L .1*L -.1*L 0.1*L -1.2*L 0.1*L]);
    view(0,15);
    zoom(1.2);
    drawnow();
    background();
end

%% setup controller
function tau = Controller(mdl,Qd)
t   = mdl.Log.t;
q   = mdl.Log.q;
dq  = mdl.Log.dq;

if t < 5
    qd = Qd(:,1);
else
    qd = Qd(:,2);
end

KT  = mdl.Log.EL.K;
Kd  = mdl.Log.EL.M;
tau = mdl.Log.dUdq - KT*(q - qd) - Kd*dq;
end

%% setup rig
function rig = setupRig(M,L,Modes)
gmdl1 = Gmodel('Pneulink.stl','ShowProcess',0);
gmdl2 = gmdl1.copy();
gmdl3 = gmdl1.copy();

gmdl1 = Blender(gmdl1,'Loft',[1.00,0.96]);
gmdl2 = Blender(gmdl2,'Loft',[0.96,0.92]);
gmdl3 = Blender(gmdl3,'Loft',[0.92,0.88]);

gmdl4 = Gmodel('SoftGripperRedux.stl','ShowProcess',0);
gmdl4 = Blender(gmdl4,'Scale',0.88);

N = 200;
X = linspace(0,L,N)';
Y = GenerateFunctionSpace(X,N,M,L);

shp = Shapes(Y,Modes,'L0',L);
 
rig = Rig(@(x) shp.string(x),'Domain',L,'XTangent',true);

rig = rig.add(gmdl1,gmdl2,gmdl3,gmdl4);

rig = rig.parent(1,0,0);
rig = rig.parent(1,1,0.3);
rig = rig.parent(2,0,0.3);
rig = rig.parent(2,1,.6);
rig = rig.parent(3,0,.6);
rig = rig.parent(3,1,.9);
rig = rig.parent(4,1,.9);

rig.g0 = SE3(roty(-pi),zeros(3,1));
rig = rig.render();
end
```


[**[1]**](https://www.festo.com/group/en/cms/10241.htm) **Bionic Handling Assistant** a soft robotic manipulator from Festo.
{: .fs-3} 

[**[2]**](https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process) **Gram-Schmidt process** on constructing a orthonormal basis.
{: .fs-3} 

