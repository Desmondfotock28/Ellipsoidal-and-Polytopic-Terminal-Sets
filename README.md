# Ellipsoidal-and-Polytopic-Terminal-Sets

This project explores the impact of terminal sets in Model Predictive Control (MPC) for a spring-mass-damper system with short prediction horizons. The system is discretized, and MPC controllers are implemented using both **polytopic** and **ellipsoidal** terminal sets.  

The repository analyzes controller performance in terms of stability, constraint satisfaction, computational efficiency, and closed-loop control behavior. Polytopic terminal sets are represented with linear inequalities and are simpler to compute, while ellipsoidal sets provide a more compact state-space representation at the cost of more complex optimization.  

This study highlights how the choice of terminal set influences MPC performance, providing insights for applications in control of multivariable and constrained systems.  
