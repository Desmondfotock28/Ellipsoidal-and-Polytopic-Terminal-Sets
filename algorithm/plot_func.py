
import numpy as np
import time
from typing import Callable
from casadi import *
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
from control import dare
from scipy.linalg import null_space
from matplotlib import rcParams



# Configure PGF for LaTeX export
rcParams.update({
    "pgf.texsystem": "pdflatex",  # Use pdflatex or xelatex
    "text.usetex": True,          # Enable LaTeX text rendering
    "font.family": "serif",       # Match LaTeX document fonts
    "pgf.preamble": [
        r"\usepackage{amsmath}",  # Use additional LaTeX packages if needed
    ]
})






import numpy as np
import matplotlib.pyplot as plt

def plot_traj_comparison(X_traj, U_traj, X_traj_ts, U_traj_ts, 
                         X_ub=None, U_ub=None):
    """
    Compare trajectories without and with terminal set constraints.

    Parameters:
    - X_traj: State trajectory without terminal set (T x nx)
    - U_traj: Control trajectory without terminal set (T x nu)
    - X_traj_ts: State trajectory with terminal set (T_ts x nx)
    - U_traj_ts: Control trajectory with terminal set (T_ts x nu)
    - X_ub: Upper bounds on states (nx,) or scalar (optional)
    - U_ub: Upper bound on inputs (scalar or array-like of shape (nu,)) (optional)
            Lower bound is assumed symmetric: -U_ub
    """

    # Convert to numpy arrays
    X_traj = np.array(X_traj).squeeze()
    U_traj = np.array(U_traj).squeeze()
    X_traj_ts = np.array(X_traj_ts).squeeze()
    U_traj_ts = np.array(U_traj_ts).squeeze()

    nx = X_traj.shape[1]

    # --- 1. State trajectories side by side ---
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), sharey=True)

    for i in range(nx):
        axes[0].plot(X_traj[:, i], label=f'State x{i+1}')
        axes[1].plot(X_traj_ts[:, i], label=f'State x{i+1}')

        # Plot state upper bounds if provided
        if X_ub is not None:
            ub_val = X_ub[i] if np.ndim(X_ub) > 0 else X_ub
            axes[0].axhline(ub_val, color='r', linestyle='--', linewidth=1, label=f'Constraint x{i+1}')
            axes[1].axhline(ub_val, color='r', linestyle='--', linewidth=1, label=f'Constraint x{i+1}')

    axes[0].set_title('States (no terminal set)')
    axes[1].set_title('States (with terminal set)')
    for ax in axes:
        ax.set_xlabel('Time step')
        ax.set_ylabel('State value')
        ax.grid(True)
        ax.legend()

    plt.tight_layout()
    plt.show()

    # --- 2. Control inputs side by side ---
    fig, axes = plt.subplots(1, 2, figsize=(14, 4), sharey=True)

    axes[0].step(range(len(U_traj)), U_traj, where='post', label='u')
    axes[1].step(range(len(U_traj_ts)), U_traj_ts, where='post', label='u')

    # Plot input constraints if provided
    if U_ub is not None:
        ub_val = U_ub if np.ndim(U_ub) == 0 else U_ub[0]
        lb_val = -ub_val

        for ax in axes:
            ax.axhline(lb_val, color='r', linestyle='--', linewidth=1, label='u lower bound')
            ax.axhline(ub_val, color='r', linestyle='--', linewidth=1, label='u upper bound')

    axes[0].set_title('Control (no terminal set)')
    axes[1].set_title('Control (with terminal set)')
    for ax in axes:
        ax.set_xlabel('Time step')
        ax.set_ylabel('Control input')
        ax.grid(True)
        ax.legend()

    plt.tight_layout()
    plt.show()

    # --- 3. Phase plot (x1 vs x2) side by side ---
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)

    axes[0].plot(X_traj[:, 0], X_traj[:, 1], marker='o')
    axes[1].plot(X_traj_ts[:, 0], X_traj_ts[:, 1], marker='o')

    axes[0].set_title('Phase plot (no terminal set)')
    axes[1].set_title('Phase plot (with terminal set)')
    for ax in axes:
        ax.set_xlabel('x1 (Position)')
        ax.set_ylabel('x2 (Velocity)')
        ax.grid(True)

    plt.tight_layout()
    plt.show()


ub_x=  np.array([1, 1])
ub_u = np.array([0.68])


controls = np.load('U_optimal.npy')
states  = np.load('X_optimal.npy')
controls_ts = np.load('U_optimal_ts.npy')
states_ts  = np.load('X_optimal_ts.npy')
#t_p = np.load('time_full.npy')

plot_traj_comparison(states, controls, states_ts, controls_ts, ub_x, ub_u)
