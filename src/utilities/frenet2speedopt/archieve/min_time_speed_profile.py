# min_time_speed_profile.py
import numpy as np
import cvxpy as cp

def min_time_speed_profile(
    s: np.ndarray,           # cumulative arc length size N
    kappa: np.ndarray,       # curvature at segments size N
    mu: float = 0.9,         # tire-road friction coefficient
    g: float = 9.81,
    a_acc: float = 3.0,      # max longitudinal accel (m/s^2)
    a_dec: float = 5.0,      # max braking decel (m/s^2)
    v_min: float = 0.5,
    v_max: float = 15.0
):
    N = len(s)
    ds = np.diff(s)          # size N-1
    k = kappa[:-1]           # segment curvature
    u = cp.Variable(N-1)     # u_i = v_i^2 at segment i

    # objective: sum(ds / sqrt(u))
    eps = 1e-6
    obj = cp.Minimize(cp.sum(ds / cp.sqrt(u + eps)))

    constraints = []

    # bounds from v_min/max
    constraints += [u >= v_min**2, u <= v_max**2]

    # lateral acceleration (friction circle simplified)
    alat_max = mu * g
    constraints += [cp.abs(k) * u <= alat_max]

    # longitudinal accel/brake constraints on u differences
    # u_{i+1} - u_i <= 2 a_acc ds_i   and   u_i - u_{i+1} <= 2 a_dec ds_i
    for i in range(N-2):
        constraints += [u[i+1] - u[i] <= 2 * a_acc * ds[i]]
        constraints += [u[i] - u[i+1] <= 2 * a_dec * ds[i]]

    prob = cp.Problem(obj, constraints)
    prob.solve(solver=cp.OSQP, eps_abs=1e-6, eps_rel=1e-6, verbose=True)

    v = np.sqrt(np.maximum(u.value, 0.0))
    return v, prob.value
