# Ellipsoidal-and-Polytopic-Terminal-Sets

## Introduction
This project investigates the impact of terminal sets in Model Predictive Control (MPC) for a spring-mass-damper system with short prediction horizons. MPC controllers are implemented using both **polytopic** and **ellipsoidal** terminal sets.  

The study analyzes performance in terms of stability, constraint satisfaction, computational efficiency, and closed-loop behavior, highlighting how the choice of terminal set influences overall MPC performance.


## System Dynamics
The spring-mass-damper system is modeled and discretized for MPC implementation. The discrete time dynamics read as follow: 

```math
x_{k+1} = A x_k + B u_k
```
where:  
- $x_k$ is the state vector at time step \(k\) (displacement and velocity),  
- $u_k$ is the control input,  
- $A$ and $B$ are the discrete-time system matrices determine based on the parameters
$m$(mass), $c$ (spring constant), $d$ (damping factor), and $∆t$ (sampling time) as:
```math
A = 
\begin{bmatrix}
1 & \Delta t \\
-\frac{c}{m} \Delta t & 1 - \frac{d}{m} \Delta t
\end{bmatrix}, 
\quad
B = 
\begin{bmatrix}
0 \\
\frac{\Delta t}{m_0}
\end{bmatrix}
```



## MPC Controller Design
MPC controllers are formulated with short prediction horizons and incorporate terminal sets to ensure stability and feasibility: 

The quadratic cost function to be minimized is:
```math
J = \sum_{k=0}^{N-1} x_k^\top Q x_k + \sum_{k=0}^{N-1} u_k^\top R u_k + x_N^\top P x_N
```
with weighting matrices:
```math
Q =\begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}, 
\quad
R = 1
```
The Discrete-time Algebraic Riccati equation (DARE) is solved to obtain $P$:
```math
P = A_d^\top P A_d - A_d^\top P B_d (R + B_d^\top P B_d)^{-1} B_d^\top P A_d + Q
```
The LQR gain $K$ is computed as:
```math
K = (R + B_d^\top P B_d)^{-1} B_d^\top P A_d
```
- **Constraints:**
 State and input constraints are defined as:
```math
-1 \le x_1 \le 1, \quad -1 \le x_2 \le 1, \quad -0.68 \le F_{\text{ext}} \le 0.68
```
- **Polytopic Terminal Sets:**
  A polytopic terminal set is a convex polytope in the state space that serves as a target set for the system states at the end of a finite time horizon in MPC. Incorporating this set helps guarantee that the system states will remain within a specified region, thereby improving stability and performance. For a spring-mass-damper system, this can mean more precise control of oscillations and damping behavior. The terminal set is constructed iteratively by computing the maximum invariant set for the closed-loop system.
  
Given the  state and input constraint sets:
```math
\mathcal{X} := \{ x \mid F x \le 1 \}, \quad \mathcal{U} := \{ u \mid G u \le 1 \}
```
The maximal positively invariant set is described by
```math
\Omega = \left\{ x \;\middle|\; F(A + BK)^i x \le 1, \; G K (A + BK)^i x \le 1, \; i = 0, \dots, n_f \right\}
```
where $n_f$ is the smallest integer such that
```math
F (A + BK)^{n_f + 1} x \le 1, \quad G K (A + BK)^{n_f + 1} x \le 1
```
hold for all $x$ in $\Omega$
  
- **Ellipsoidal Terminal Sets:** Ellipsoids are very popular as candidate invariant sets.
The ellipsoidal set is defined as:
```math
E_z = \{ z \;|\; z^\top P_z z \le 1 \}
```
It can be determined by solving the semidefinite program for the positively invariant ellipsoidal set:
```math
\begin{aligned}
& \max_{S, H} \log \det(S_{xx}) \\
& \text{subject to} \\
\begin{bmatrix}
S \Psi S^\top & \Psi S S \\
(\Psi S S)^\top & \cdot
\end{bmatrix} \succeq 0
\end{aligned}
```

The MPC optimization problem is solved at each time step to compute optimal control inputs while respecting system constraints.

## Simulation Results
Simulations compare the performance of MPC controllers with polytopic and ellipsoidal terminal sets. Key observations include:  

- Impact on system stability and closed-loop response.  
- Constraint satisfaction during control.  
- Computational efficiency and solver performance.  

Visualization scripts generate plots showing state trajectories, control inputs, and terminal set effects for comparison.


