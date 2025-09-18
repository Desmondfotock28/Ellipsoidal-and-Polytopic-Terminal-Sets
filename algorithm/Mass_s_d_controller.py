import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from control import  dare

# Define system parameters
m = 3.0 
d = 0.9
c = 1.8

# Discretization parameters
dt = 0.1

# Continuous-time system matrices
A_c = np.array([[0, 1], [-c/m, -d/m]])
B_c = np.array([[0], [1/m]])
print(A_c)

# Discrete-time system matrices using matrix exponential
A_d = np.eye(2) + dt * A_c
B_d = dt * B_c

nx = A_d.shape[1]
nu = B_d.shape[1]

#simulating system without control 

def xdot(x,u):
    return np.dot(A_c,x)+np.dot(B_c,u)

def integrateOpenLoop(x0, U, steps, dt=1e-3):
    X = np.empty([x0.size, steps+1])
    X[:,0]=x0
    for t in range(steps):
        x = X[:, t]               #current state
        u = U[:, t]               #current control
        xnext = x + xdot(x,u)*dt  #Euler integrator
        X[:, t+1] = xnext
    return X

# integrate dynamics with 0 control input

steps = 200

x0 = np.array([0.68,0.68])

U_op = np.zeros([1, steps])


X_op=integrateOpenLoop(x0,U_op, steps, dt)

legend = ["Position", "Velocity"]

for n in range(X_op.shape[0]):
    plt.plot(X_op[n,:],label="%s"%legend[n])
plt.legend(loc=1)
plt.grid(True)
plt.show()

# Define the CasADi system function using discrete-time matrices
x = SX.sym("x", nx)
u = SX.sym("u", nu)

x_next = A_d @ x + B_d @ u

# Create the CasADi function
system = Function("sys", [x, u], [x_next])

# Define initial state
x0 = np.array([0.68, 0.68]).reshape(2, 1)

# Define cost function parameters
Q = 1
Q = Q * np.diag(np.ones(nx))

R = 1
R = np.diag(R * np.ones(nu))

S, L, G = dare(A_d, B_d, Q, R)
# Terminal cost (solution to Riccati equation)

# Define the stage cost and terminal cost
stage_cost = bilin(Q, x) + R * u**2
terminal_cost = bilin(Q, x)

stage_cost_fcn = Function("cost", [x, u], [stage_cost])
terminal_cost_fcn = Function("T_cost", [x], [terminal_cost])

# State constraints
lb_x = -1* np.ones((nx, 1))
ub_x =  1* np.ones((nx, 1))

# Input constraints
lb_u = -0.68 * np.ones((nu, 1))
ub_u = 0.68 * np.ones((nu, 1))

# discretisation
N = 2

X = SX.sym("X", (N + 1) * nx, 1)
U = SX.sym("U", N * nu, 1)

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

    
def optimize_trajectory(X, U, N, nx, nu, stage_cost_fcn, system, lb_x, ub_x, lb_u, ub_u):
    J = 0
    g = [ ]
    lb_g = [ ]
    ub_g = [ ]
    lb_X = [ ] 
    ub_X = [ ] 
    lb_U = [ ] 
    ub_U = [ ]

    for k in range(N): 
        x_k = X[k * nx:(k + 1) * nx]
        x_k_next = X[(k + 1) * nx:(k + 2) * nx]
        u_k = U[k * nu:(k + 1) * nu]

        J += stage_cost_fcn(x_k, u_k)  # objective
        
        # equality constraints (system equation)
        x_k_next_calc = system(x_k, u_k)

        g.append(x_k_next - x_k_next_calc)

        lb_g.append(np.zeros((nx, 1)))

        ub_g.append(np.zeros((nx, 1)))

        #states Constraints 
        
        lb_X.append(lb_x)

        ub_X.append(ub_x)

        lb_U.append(lb_u)

        ub_U.append(ub_u)

    return J, g, lb_g, ub_g, lb_X, ub_X, lb_U, ub_U

J, g, lb_g, ub_g, lb_X, ub_X, lb_U, ub_U = optimize_trajectory(X, U, N, nx, nu, stage_cost_fcn, system, lb_x, ub_x, lb_u, ub_u)

x_N = X[N * nx : (N + 1) * nx,:]

J += terminal_cost_fcn(x_N)

lb_X.append(lb_x)
ub_X.append(ub_x)


# Concatenate X and U into a single optimization variable vector
x = vertcat(X, U)

# Concatenate the bounds for X and U
lbx = vertcat(*lb_X, *lb_U)
ubx = vertcat(*ub_X, *ub_U)

# Concatenate constraints and their bounds
g_conc = vertcat(*g)
lbg = vertcat(*lb_g)
ubg = vertcat(*ub_g)

# Define the optimization problem
prob = {'f': J, 'x': x, 'g': g_conc}

# Create the NLP solver instance
solver = nlpsol('solver', 'ipopt', prob)


# Set the initial state
lbx[:nx]=x0

ubx[:nx]=x0


# Solve the problem
solver_input = {'lbx': lbx, 'ubx': ubx, 'lbg': lbg, 'ubg': ubg}

solver_output = solver(**solver_input)

# Extract the optimal solution
x_opt = solver_output['x'] 

#simulation time 
Nmpc=50


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


X_traj, U_traj = run_closed_loop_mpc(x0, Nmpc, lbx, ubx, lbg, ubg, solver, system)

#plot trajectories 

plot_traj(X_traj, U_traj)

