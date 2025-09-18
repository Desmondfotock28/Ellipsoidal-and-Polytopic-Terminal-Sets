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



## Controller Design
MPC controllers are formulated with short prediction horizons and incorporate terminal sets to ensure stability and feasibility:  

- **Polytopic Terminal Sets:** Represented by linear inequalities, simple to compute.  
- **Ellipsoidal Terminal Sets:** Compact representation of the state space, but involve more complex optimization.  

The MPC optimization problem is solved at each time step to compute optimal control inputs while respecting system constraints.

## Simulation Results
Simulations compare the performance of MPC controllers with polytopic and ellipsoidal terminal sets. Key observations include:  

- Impact on system stability and closed-loop response.  
- Constraint satisfaction during control.  
- Computational efficiency and solver performance.  

Visualization scripts generate plots showing state trajectories, control inputs, and terminal set effects for comparison.


