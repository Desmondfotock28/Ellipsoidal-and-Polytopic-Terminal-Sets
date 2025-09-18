import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from control import  dare
from pypoman import compute_polytope_vertices
from pypoman.polygon import plot_polygon
from numpy.linalg import matrix_power as mp
import time

start_time = time.time()


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

def run_closed_loop_mpc(x0, Nmpc, lbx, ubx, lbg, ubg, solver, system):
    
    # initial state
    x_k = x0
    nx = x0.shape[0]

    # xxx
    X_traj = [x0]
    U_traj = []
    for _ in range(Nmpc):

        # update initial condition
        lbx[:nx]=x_k
        ubx[:nx]=x_k

        # solve MPC problem
        res = solver(lbx=lbx,ubx=ubx,lbg=lbg,ubg=ubg)

        # extract optimal input
        u_k = res['x'][(N+1)*nx:(N+1)*nx+nu,:]

        #print( u_k )

        # simulate system
        x_k = system(x_k,u_k)

        # update data lists
        X_traj.append(x_k)
        U_traj.append(u_k)
        
    return X_traj, U_traj


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
F = np.array([[ 1, 0],
              [-1, 0],
              [ 0, 1],
              [ 0,-1]])

# input constraints matrix
G = np.array([[ 25/17],
              [-25/17]])

# state feedback matrix
#K = np.array([[-1, -1]])

# closed-loop system
A_cl = A_d+B_d@K

#eigenvalues of close loop
print(np.linalg.eig(A_cl)[0])


# Maximum number of iterations
max_iter = 100

# Initialize matrix describing the invariant set
invariant_mat = np.zeros((0,2))
# run loop
for n_f in range(max_iter):
    
    # extend the matrix describing the invariant set
    invariant_mat = np.vstack([invariant_mat, F @ mp(A_cl,n_f), G @ K @ mp(A_cl,n_f)])
    
    # termination criterion
    one_vec = np.ones((invariant_mat.shape[0],1))
    
    # compute vertices of current iterate of the maximum invariant set
    verts = compute_polytope_vertices(invariant_mat,one_vec)
    
    # compute predecessor states of the current vertices
    verts_next = [A_cl @ np.reshape(vert,(-1,1)) for vert in verts]
    
    # check if all verts lie inside the current iterate of the maximum invariant set
    in_omega = [all(invariant_mat @ vert <= one_vec) for vert in verts_next]
    
    # if all predecessor verts inside the current iterate of the maximum invariant set -> break
    if all(in_omega):
        print('Algorithm converged after ' + str(n_f+1) + ' step(s).')
        break

plot_polygon(verts,color='r',alpha=0.4)
plt.ylim(-1,1)
plt.xlim(-1.1,1.1)
plt.show()

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

# Add terminal set constraints
g.append(invariant_mat@x_terminal)

for i in range(invariant_mat.shape[0]):
    lb_g.append(-np.inf)
    ub_g.append(1)


g = vertcat(*g)
lbg = vertcat(*lb_g)
ubg = vertcat(*ub_g)

sim_steps= 350
# solver creation
prob = {'f':J,'x':xu,'g':g}
solver_with_terminal_set = nlpsol('solver','ipopt',prob)

X_traj, U_traj = run_closed_loop_mpc(x0, sim_steps, lbxu, ubxu, lbg, ubg,  solver_with_terminal_set, system)

end_time = time.time()
elapsed_time = end_time - start_time
print(f"Elapsed time: {elapsed_time} seconds")

np.save("X_optimal_ts",X_traj)
np.save("U_optimal_ts",U_traj)

plot_traj(X_traj, U_traj)

