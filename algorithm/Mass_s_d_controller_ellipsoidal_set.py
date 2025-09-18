import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from control import  dare
import cvxpy as cp
import time



#Function definition
def plot_ellipsoid(P_x):
    # Perform Cholesky decomposition of P_x
    L = np.linalg.cholesky(P_x)

    # Generate points on the unit circle
    theta = np.linspace(0, 2 * np.pi, 100)
    circle_points = np.vstack((np.cos(theta), np.sin(theta)))

    # Transform points to the ellipsoid
    ellipsoid_points = np.linalg.inv(L).dot(circle_points)

    # Plotting
    fig, ax = plt.subplots()

     # Plot the ellipsoid
    ax.fill(ellipsoid_points[0, :], ellipsoid_points[1, :], color='b', alpha=0.6)

    # Plot settings
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Ellipsoid defined by $x^T P_x x \leq 1$')
    ax.set_aspect('equal', 'box')
    ax.grid(True)
    plt.show()

def plot_traj(X_traj, U_traj):
    # Extract the state trajectories
    X_traj = np.array(X_traj).squeeze()
    U_traj = np.array(U_traj).squeeze()

# Plot the state trajectories
    plt.figure(figsize=(12, 8))
    plt.plot(X_traj[:, 0], label='Position')
    plt.plot(X_traj[:, 1], label='Velocity')
    for i in range(2, nx):
        plt.plot(X_traj[:, i], label=f'State x{i+1}')
    plt.xlabel('Time step')
    plt.ylabel('State value')
    plt.title('State Trajectories')
    plt.legend()
    plt.grid(True)
    plt.show()

# Plot the control inputs as a step plot
    plt.figure(figsize=(12, 4))
    plt.step(range(len(U_traj)), U_traj, where='post', label='Control input u')
    plt.xlabel('Time step')
    plt.ylabel('Control input value')
    plt.title('Control Input Trajectory')
    plt.legend()
    plt.grid(True)
    plt.show()

# Plot state 2 against state 1
    plt.figure(figsize=(8, 6))
    plt.plot(X_traj[:, 0], X_traj[:, 1], marker='o')
    plt.xlabel('Position (State 1)')
    plt.ylabel('Velocity (State 2)')
    plt.title('State 2 vs State 1')
    plt.grid(True)
    plt.show()

def plot_traj1(X_traj, U_traj):
    # Extract the state trajectories
    X_traj = np.array(X_traj).squeeze()
    U_traj = np.array(U_traj).squeeze()
    
    nx = X_traj.shape[1]  # Number of states
    
    # Plot the state trajectories
    plt.figure(figsize=(16, 6))
    
    # Subplot 1: State trajectories
    plt.subplot(1, 2, 1)
    for i in range(nx):
        plt.plot(X_traj[:, i], label=f'State x{i+1}')
    plt.xlabel('Time step')
    plt.ylabel('State value')
    plt.title('State Trajectories')
    plt.legend()
    plt.grid(True)
    
    # Subplot 2: State 2 vs State 1
    plt.subplot(1, 2, 2)
    plt.plot(X_traj[:, 0], X_traj[:, 1], marker='o')
    plt.xlabel('Position (State 1)')
    plt.ylabel('Velocity (State 2)')
    plt.title('State 2 vs State 1')
    plt.grid(True)
    
    # Adjust layout to prevent overlap
    plt.tight_layout()
    
    # Show plots
    plt.show()

    # Plot the control inputs as a step plot
    plt.figure(figsize=(12, 4))
    plt.step(range(len(U_traj)), U_traj, where='post', label='Control input u')
    plt.xlabel('Time step')
    plt.ylabel('Control input value')
    plt.title('Control Input Trajectory')
    plt.legend()
    plt.grid(True)
    plt.show()

def run_closed_loop_mpc(x0, Nmpc, lbx, ubx, lbg, ubg, solver, system):
    
    # initial state
    x_k = x0
    nx = x0.shape[0]
    # xxx
    X_traj = [x0]
    U_traj = []
    t_comp = []
    for _ in range(Nmpc):

        # update initial condition
        lbx[:nx]=x_k
        ubx[:nx]=x_k

        # solve MPC problem
        start_time = time.time()
        res = solver(lbx=lbx,ubx=ubx,lbg=lbg,ubg=ubg)
        end_time = time.time()
        elapsed_time = end_time - start_time
        t_comp.append(elapsed_time)
        # extract optimal input
        u_k = res['x'][(N+1)*nx:(N+1)*nx+nu,:]

        #print( u_k )

        # simulate system
        x_k = system(x_k,u_k)

        # update data lists
        X_traj.append(x_k)
        U_traj.append(u_k)
        
    t_comp = np.array(t_comp)    
    return X_traj, U_traj, t_comp

# Define system parameters
m = 3.0 
d = 0.9
c = 1.8

# Discretization parameters
dt = 0.1

# Continuous-time system matrices
A_c = np.array([[0, 1], [-c/m, -d/m]])
B_c = np.array([[0], [1/m]])


# Discrete-time system matrices using matrix exponential
A_d = np.eye(2) + dt * A_c
B_d = dt * B_c

nx = A_d.shape[1]
nu = B_d.shape[1]

# Define the CasADi system function using discrete-time matrices
x = SX.sym("x", nx)
u = SX.sym("u", nu)

x_next = A_d @ x + B_d @ u

# Create the CasADi function
system = Function("sys", [x, u], [x_next])

# Define initial state
x0 = np.array([0.68, 0.68]).reshape(2, 1)

# Define cost function parameters
#Q = 5
#Q = Q * np.diag(np.ones(nx))

Q= np.array([[1, 0],[0 , 1]])

R = 1
R = np.diag(R * np.ones(nu))

P, L, K = dare(A_d, B_d, Q, R)
# Terminal cost (solution to Riccati equation)

# Define the stage cost and terminal cost
stage_cost = bilin(Q, x) + R * u**2
terminal_cost = bilin(Q, x)

stage_cost_fcn = Function("cost", [x, u], [stage_cost])
terminal_cost_fcn = Function("T_cost", [x], [terminal_cost])

#stability Analysis of MPC: 
# convert to numpy-array
P = np.array(P)

# convert to numpy-array
K = -np.array(K)


# state constraints matrix
F = np.array([[ 1, 0],[-1, 0],[ 0, 1],[ 0,-1],[ 0,0],[ 0,0]])

# input constraints matrix
G = np.array([[0], [0],[0], [0],[ 25/17],[-25/17]])

# state feedback matrix
#K = np.array([[-1, -1]])

# closed-loop system
A_cl = A_d+B_d@K

#eigenvalues of close loop
print(np.linalg.eig(A_cl)[0])

#compute ellipsoidal terminal matrix 
N_e = 2
nc = 6
# Define block partition matrices
I_nu = np.eye(nu)

# Correct definition of E matrix
E = np.hstack([I_nu] + [np.zeros((nu, nu)) for _ in range(N_e - 1)])

# Define M matrix correctly
M = np.zeros((N_e * nu, N_e * nu))
for i in range(1, N_e):
    M[i * nu - nu:(i + 1) * nu - nu, i * nu:i * nu + nu] = I_nu

# Define Psi matrix correctly
Psi_top_left = A_d + B_d @ K
Psi_top_right = B_d @ E
Psi_bottom_left = np.zeros((N_e * nu, nx))
Psi_bottom_right = M

Psi = np.block([
    [Psi_top_left, Psi_top_right],
    [Psi_bottom_left, Psi_bottom_right]
])

# Define optimization variables
S = cp.Variable((nx + N_e * nu, nx + N_e * nu), symmetric=True)
H = cp.Variable(( nc,  nc), symmetric=True)

# Objective function
objective = cp.Maximize(cp.log_det(S[:nx, :nx]))

# Define intermediate matrix L
L = S@np.block([
    [(F + G @ K).T],
    [(G @ E).T]
])

# Constraints
constraints = [
    S >> 0,
    H >> 0,
    cp.bmat([
        [S, S @ Psi],
        [Psi.T @ S, S]
    ]) >> 0,
    
    cp.bmat([
        [H, L.T],
        [L, S]
    ]) >> 0
]

# Identity matrix for bounding H
identity_matrix = np.eye(nc)
for i in range(nc):
    e_i = identity_matrix[:,i]
    constraints.append(e_i.T @ H @ e_i <= 1)
# Solve the problem
problem = cp.Problem(objective, constraints)
problem.solve()

# Output results
if problem.status == cp.OPTIMAL:
    print("Optimal value:", problem.value)
    print("S matrix:", S.value)
    print("H matrix:", H.value)
else:
    print("Problem status:", problem.status)

# Construct I_nx and 0 matrices
I_nx = np.eye(nx)
    
zero_matrix = np.zeros((nx, N_e * nu))
    
    # Create the matrix [I_nx 0]
I_nx_0 = np.hstack([I_nx, zero_matrix])
    
    # Create the matrix [I_nx; 0]
I_nx_col_0 = np.vstack([I_nx, zero_matrix.T])
    
    # Compute the product
S_submatrix = I_nx_0 @ S.value @ I_nx_col_0
    
    # Compute the inverse of the submatrix
P_x = np.linalg.inv(S_submatrix)
print("P_x matrix:", P_x)

plot_ellipsoid(P_x)

#Design MPC with ellipsoidal set 
#Horizon length
N=2
# Optimization variables
X = SX.sym("X",(N+1)*nx,1)
U = SX.sym("U",N*nu,1)

# Initialize palceholders
J = 0
g = []
lb_g = []
ub_g = []
lb_x = []
ub_x = []
lb_u = []
ub_u = []

# Loop for problem construction
for k in range(N):
    # 01 - Your code here!
    x_k = X[k*nx:(k+1)*nx,:]
    x_k_next = X[(k+1)*nx:(k+2)*nx,:]
    u_k = U[k*nu:(k+1)*nu,:]
    # 01

    # 02 - Your code here!
    # objective
    J += stage_cost_fcn(x_k,u_k)
    # 02
    
    # 03 - Your code here!
    # equality constraints (system equation)
    x_k_next_calc = system(x_k,u_k)
    lb_g.append(np.zeros((nx,1)))
    ub_g.append(np.zeros((nx,1)))
    g.append(x_k_next - x_k_next_calc)
    # 03
    # input constraints
    lb_u.append(-0.68)
    ub_u.append(0.68)

     # state constraints
    lb_x.append(-np.ones((nx,1)))
    ub_x.append(np.ones((nx,1)))
    
# Terminal cost
x_terminal = X[N*nx:(N+1)*nx,:]
J += terminal_cost_fcn(x_terminal)

# add terminal constraint
lb_x.append(-np.ones((nx,1)))
ub_x.append(np.ones((nx,1)))

# optimization variables
xu = vertcat(X,U)

# bounds on optimization variables
lbx = vertcat(*lb_x)
ubx = vertcat(*ub_x)
lbu = vertcat(*lb_u)
ubu = vertcat(*ub_u)
lbxu = vertcat(lbx,lbu)
ubxu = vertcat(ubx,ubu)

# add terminal constraint
g.append(x_terminal.T@P_x@x_terminal)
lb_g.append(-np.inf)
ub_g.append(1)

g = vertcat(*g)

lbg = vertcat(*lb_g)
ubg = vertcat(*ub_g)

sim_steps=350
# solver creation
prob = {'f':J,'x':xu,'g':g}
solver_with_terminal_set = nlpsol('solver','ipopt',prob)

X_traj, U_traj, t_comp= run_closed_loop_mpc(x0, sim_steps, lbxu, ubxu, lbg, ubg,  solver_with_terminal_set, system)

t_mean = np.mean(t_comp)

print(t_mean)
np.save("X_optimal_Es",X_traj)
np.save("U_optimal_Es",U_traj)


plot_traj(X_traj, U_traj)
  
