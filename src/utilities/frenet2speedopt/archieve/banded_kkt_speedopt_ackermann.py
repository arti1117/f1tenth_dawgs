"""
banded_kkt_speedopt_ackermann.py

Friction-circle for 4WD Ackermann vehicle + banded KKT + sparse LU interior-point solver.

Dependencies:
  pip install numpy scipy

Notes / Simplifications:
 - p = 2 (path-tangent, path-normal generalized forces)
 - r = 8 (4 wheels * [Fx, Fy])
 - mtilde = mass m applied equally to both generalized DOFs
 - ctilde = dtilde = 0 (no Coriolis/gravity terms in RHS)
 - friction: per-wheel ||u_wheel|| <= F_max (we set F_max = mu * (m*g)/4 by default)
 - Barrier method used for inequalities; equality constraints kept explicit and solved via KKT.
 - Newton linear system is the KKT block system; assembled as sparse and solved with scipy.sparse.linalg.splu.
"""

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

# ----------------------
# Utility geometry helpers
def normalize(v, eps=1e-12):
    n = np.linalg.norm(v)
    if n < eps:
        return v * 0.0, 0.0
    return v / n, n

def rotation_matrix(theta):
    c = np.cos(theta); s = np.sin(theta)
    return np.array([[c, -s],[s, c]])

# ----------------------
# Build Rtilde for 4 wheels at a midpoint
def Rtilde_ackermann_4wd(s_prime_midpoint, steering_delta, wheel_positions_body, yaw_of_vehicle=0.0):
    """
    Construct Rtilde (p x r) mapping wheel forces (Fx,Fy for each wheel) to generalized
    forces [F_tangent; F_normal] in path frame.
    Inputs:
      s_prime_midpoint: tangent vector (2D) of path at midpoint (world coords)
      steering_delta: scalar steering angle (front axle) (radians)
      wheel_positions_body: list of wheel pos in body frame (4 x 2) describing [x_forward, y_right]
                           order: [front_left, front_right, rear_left, rear_right]
      yaw_of_vehicle: angle of vehicle body relative to world x-axis (radians) - here we use tangent angle
    Returns:
      R: (2 x 8) numpy array
    """
    # path tangent and normal (world)
    t_unit, _ = normalize(s_prime_midpoint)
    n_unit = np.array([-t_unit[1], t_unit[0]])  # rotate tangent by +90 deg gives normal
    # wheel steering: front wheels have delta, rear wheels have 0
    steering_angles = [steering_delta, steering_delta, 0.0, 0.0]
    R = np.zeros((2, 8))
    for i in range(4):
        delta = steering_angles[i]
        # wheel force in wheel-local coords [Fx (forward), Fy (lateral)] -> convert to world
        # wheel world rotation = yaw_of_vehicle + delta
        rot = rotation_matrix(yaw_of_vehicle + delta)
        # force basis columns for Fx and Fy in world coords
        fx_world = rot @ np.array([1.0, 0.0])  # unit Fx
        fy_world = rot @ np.array([0.0, 1.0])  # unit Fy
        # projection onto tangent and normal
        R[:, 2*i + 0] = np.array([ np.dot(t_unit, fx_world), np.dot(n_unit, fx_world) ])
        R[:, 2*i + 1] = np.array([ np.dot(t_unit, fy_world), np.dot(n_unit, fy_world) ])
    return R  # shape (2,8)

# ----------------------
# Objective (time) terms and derivatives for b variables
def time_obj_and_derivs(b, dtheta):
    """
    obj = sum_{i=0..n-1} 2 dtheta / (sqrt(b_i) + sqrt(b_{i+1}))
    returns:
      obj_val, grad_b (len n+1), Hessian_b (sparse tridiagonal) as three diagonals (lower, diag, upper)
    """
    n1 = len(b)
    n = n1 - 1
    s = np.sqrt(np.maximum(b, 1e-20))
    obj = 0.0
    grad = np.zeros(n1)
    # tridiagonal Hessian: we'll form three arrays: lower (n), diag (n1), upper (n)
    lower = np.zeros(n)  # H[i, i-1] for i=1..n
    diag = np.zeros(n1)
    upper = np.zeros(n)  # H[i, i+1] for i=0..n-1
    for i in range(n):
        denom = (s[i] + s[i+1])
        term = 2.0 * dtheta / denom
        obj += term
        # derivatives
        # d/ db_i of term = - dtheta * (1) / ( denom^2 * s[i] )
        d_bi = - dtheta / ( (denom**2) * s[i] )
        d_bi1 = - dtheta / ( (denom**2) * s[i+1] )
        grad[i] += d_bi
        grad[i+1] += d_bi1
        # second derivatives:
        # d2/d b_i^2 term = dtheta * ( 1/(2*b_i^(3/2)) * 2/(denom^3) ??? ) -> derive carefully
        # We'll compute exact analytic Hessian entries by symbolic-like expressions:
        # Let si = s[i], sj = s[i+1], denom = si+sj
        # d term / d bi = - dtheta * 1/(denom^2) * 1/si
        # differentiate again:
        # d2 / dbi^2 = -dtheta * [ d(1/(denom^2))/dbi * 1/si + (1/(denom^2)) * d(1/si)/dbi ]
        # d(1/(denom^2))/dbi = -2/(denom^3) * d(denom)/dbi = -2/(denom^3) * (1/(2 si))
        #             = -1/(denom^3 * si)
        # d(1/si)/dbi = d( si^{-1} )/dbi = -1 * si^{-2} * d(si)/dbi = -1 * si^{-2} * (1/(2 si)) = -1/(2 si^3)
        # combine:
        # d2 = -dtheta * [ (-1/(denom^3 * si)) * (1/si) + (1/(denom^2)) * (-1/(2 si^3)) ]
        #    = -dtheta * [ -1/(denom^3 * si^2) - 1/(2 denom^2 * si^3) ]
        #    = dtheta/( denom^3 * si^2 ) + dtheta/(2 denom^2 * si^3)
        si = s[i]
        sj = s[i+1]
        denom = si + sj
        # diagonal ii
        H_ii = dtheta / ( denom**3 * (si**2) ) + dtheta / (2.0 * denom**2 * (si**3))
        # similarly for jj
        H_jj = dtheta / ( denom**3 * (sj**2) ) + dtheta / (2.0 * denom**2 * (sj**3))
        # cross derivative d2/(dbi dbj) = derivative of d_bi1 wrt bi:
        # from symmetry we can derive:
        # d2/dbi dbj = dtheta * [ 1/(denom^3 * si * sj) - 1/(2 denom^2 * si^2 * sj) - 1/(2 denom^2 * si * sj^2) ]
        # but simpler: compute numeric via small perturbation would be robust; however we'll compute symbolic approx:
        H_ij = dtheta * ( 1.0/(denom**3 * si * sj) )
        # Accumulate into tri-diag
        diag[i] += H_ii
        diag[i+1] += H_jj
        lower[i] += H_ij
        upper[i] += H_ij
    return obj, grad, lower, diag, upper

# ----------------------
# Barrier for friction circle for each wheel u_w (2-vector)
def barrier_friction_u_and_derivs(u_vec, Fmax, tau):
    """
    u_vec: (2,) wheel force vector
    barrier term: phi = -tau * log( Fmax - ||u|| )
    returns phi, grad_u (2,), Hessian_u (2x2)
    """
    norm_u = np.linalg.norm(u_vec) + 1e-12
    slack = Fmax - norm_u
    if slack <= 0:
        # outside feasible interior; return huge penalty (should not happen if initial feasible)
        phi = 1e20
        grad = np.zeros_like(u_vec)
        H = np.eye(2) * 1e12
        return phi, grad, H
    phi = -tau * np.log(slack)
    # gradient: tau * (u / norm_u) / slack
    grad = tau * (u_vec / norm_u) / slack
    # Hessian: tau * [ (I / norm_u - (u u^T)/norm_u^3)/slack + (u u^T)/(norm_u^2 * slack^2) ]
    I = np.eye(2)
    uuT = np.outer(u_vec, u_vec)
    term1 = (I / norm_u - uuT / (norm_u**3)) / slack
    term2 = uuT / ( (norm_u**2) * (slack**2) )
    H = tau * (term1 + term2)
    return phi, grad, H

# ----------------------
# Assemble KKT and solve (single Newton step)
def assemble_kkt_and_solve(H_xx, A_eq, grad_f, r_eq):
    """
    Solve KKT system:
      [ H   A^T ] [dx] = [-grad_f]
      [ A    0  ] [dy]   [-r_eq]
    where A_eq * x = 0 are equality residuals r_eq (A_eq x - b_eq)
    Inputs:
      H_xx: (Nvar x Nvar) sparse or dense array (we'll give as scipy.sparse.csr)
      A_eq: (Meq x Nvar) sparse (csr)
      grad_f: (Nvar,) vector
      r_eq: (Meq,) residual (A_eq x - b_eq)
    Returns dx (Nvar,), dy (Meq,)
    """
    # Build KKT matrix in block sparse form
    N = H_xx.shape[0]
    M = A_eq.shape[0]
    # Top-left H, top-right A^T
    top = sp.hstack([H_xx, A_eq.transpose()]).tocsr()
    bottom = sp.hstack([A_eq, sp.csr_matrix((M,M))]).tocsr()
    K = sp.vstack([top, bottom]).tocsr()
    rhs = -np.concatenate([grad_f, r_eq])
    # Solve with sparse LU
    lu = spla.splu(K.tocsc())
    sol = lu.solve(rhs)
    dx = sol[:N]
    dy = sol[N:]
    return dx, dy

# ----------------------
# Main solver function (interior point with banded KKT)
def solve_banded_kkt_ackermann(
    s_nodes,           # (n+1, 2) path nodes (x,y)
    initial_v_scalar,  # scalar initial speed along path direction at theta0
    n_intervals,
    wheel_positions_body, # (4,2) wheel positions in body coords [x_forward, y_right]
    vehicle_mass = 1.0,
    mu = 0.9,
    g = 9.81,
    steering_delta_fun = None,   # function of midpoint index -> steering delta (radians)
    max_newton_iters = 30,
    tau0 = 1.0,
    tau_multiplier = 10.0
):
    """
    Solve discretised min-time problem with barrier method and KKT solves.
    Returns b (n+1), a (n), u (n, 8)
    """
    n_plus_1 = s_nodes.shape[0]
    assert n_plus_1 == n_intervals + 1
    n = n_intervals
    dtheta = 1.0 / n  # assume theta normalized 0..1
    # Compute tangent approximations at nodes and midpoints
    s_prime = (s_nodes[1:] - s_nodes[:-1]) / dtheta  # (n,2) -- midpoints tangent approx
    # Variables ordering: [ b(0..n) (n+1) , a(1..n) (n) , u_flat (n * 8) ]
    N_b = n + 1
    N_a = n
    r_per_mid = 8
    N_u = n * r_per_mid
    Nvar = N_b + N_a + N_u

    # equality constraints count:
    # dynamics (p*n) where p=2 -> 2*n
    # b-a relations: n eqs
    # b0 fixed: 1 eq
    Meq = 2*n + n + 1

    # Build initial feasible point (small positive b, zero a, small u)
    b0 = np.zeros(N_b)
    # set b0[0] from initial_v_scalar and first segment length
    seg_len0 = np.linalg.norm(s_nodes[1] - s_nodes[0])
    b0[0] = (initial_v_scalar * dtheta / max(seg_len0, 1e-6))**2
    # other b small positive
    b0[1:] = max(1e-4, b0[0]*0.5)
    a0 = np.zeros(N_a)
    u0 = np.zeros(N_u)  # small initial wheel forces
    # Stack into x
    x = np.concatenate([b0, a0, u0])

    # equality RHS function builder
    def equality_residuals_and_A(x_vec):
        # compute residuals r = A x - b_eq  (we keep b_eq = 0 except b0 fixed which we put as eq)
        b_vec = x_vec[0:N_b]
        a_vec = x_vec[N_b:N_b+N_a]
        u_vec = x_vec[N_b+N_a:].reshape((n, r_per_mid))
        # residuals list
        res = []
        rows = []
        cols = []
        data = []
        eq_idx = 0
        # dynamics: for i=0..n-1 (midpoints) -> R_i u_i - m*a_i - c*(b_i+b_{i+1})/2 - d = 0  (p=2)
        for i in range(n):
            # compute Rtilde at midpoint
            steering = steering_delta_fun(i) if steering_delta_fun is not None else 0.0
            R_i = Rtilde_ackermann_4wd(s_prime[i], steering, wheel_positions_body, yaw_of_vehicle = np.arctan2(s_prime[i,1], s_prime[i,0]))
            # left: R_i @ u_i
            u_i = u_vec[i]
            left = R_i @ u_i
            # right: m * a_i + c * (b_i+b_{i+1})/2 + d  (we set c=d=0)
            right = vehicle_mass * a_vec[i] * np.ones(2)
            r_dyn = left - right
            res.extend(r_dyn.tolist())
            # Build A rows: the linear mapping coefficients for this equality w.r.t x
            # For R_i * u_i: coefficients on u entries
            for p_row in range(2):
                # u indices start at offset_u = N_b + N_a
                offset_u = N_b + N_a
                # fill entries for 8 u variables of this midpoint
                for j in range(8):
                    rows.append(eq_idx + p_row)
                    cols.append(offset_u + i*8 + j)
                    data.append(R_i[p_row, j])
                # coefficient for a_i: -m
                rows.append(eq_idx + p_row)
                cols.append(N_b + i)
                data.append(-vehicle_mass)
                # coefficient for b_i and b_{i+1}: zero because c=0
            eq_idx += 2

        # b-a relations: for i=1..n  -> b_i - b_{i-1} - 2 a_i dtheta = 0
        for i in range(1, n+1):
            # row eq_idx
            rows.append(eq_idx); cols.append(i); data.append(1.0)        # b_i
            rows.append(eq_idx); cols.append(i-1); data.append(-1.0)     # -b_{i-1}
            rows.append(eq_idx); cols.append(N_b + (i-1)); data.append(-2.0*dtheta)  # -2 dtheta * a_i
            # residual value:
            res.append( b_vec[i] - b_vec[i-1] - 2.0 * a_vec[i-1] * dtheta )
            eq_idx += 1

        # b0 fixed equation: b0 - b0_val = 0
        rows.append(eq_idx); cols.append(0); data.append(1.0)
        res.append(b_vec[0] - b0[0])
        eq_idx += 1
        assert eq_idx == Meq
        # build sparse A
        A = sp.csr_matrix((data, (rows, cols)), shape=(Meq, Nvar))
        r = np.array(res)
        return A, r

    # Barrier outer loop: increase tau
    tau = tau0
    for barrier_round in range(4):  # run a few barrier multipliers
        # inner Newton iterations
        for it in range(max_newton_iters):
            # Evaluate objective, gradient and Hessian at x
            # split
            b_vec = x[0:N_b]
            a_vec = x[N_b:N_b+N_a]
            u_vec = x[N_b+N_a:].reshape((n, r_per_mid))
            # time objective part and b-derivs
            obj_time, grad_b_time, lower_b, diag_b, upper_b = time_obj_and_derivs(b_vec, dtheta)
            # barrier contributions from each wheel
            obj_bar = 0.0
            grad_u = np.zeros(N_u)
            H_u_blocks = {}  # dict i -> Hessian (8x8) for midpoint i
            Fmax_per_wheel = mu * vehicle_mass * g / 4.0
            for i in range(n):
                # for each of 4 wheels
                Hblock = np.zeros((8,8))
                grad_block = np.zeros(8)
                obj_block = 0.0
                for w in range(4):
                    u_w = u_vec[i, 2*w:2*w+2]
                    phi, g_w, H_w = barrier_friction_u_and_derivs(u_w, Fmax_per_wheel, tau)
                    obj_block += phi
                    grad_block[2*w:2*w+2] = g_w
                    Hblock[2*w:2*w+2, 2*w:2*w+2] = H_w
                obj_bar += obj_block
                H_u_blocks[i] = Hblock
                grad_u[i*8:(i+1)*8] = grad_block
            # total objective
            obj_total = obj_time + obj_bar
            # gradient full vector
            grad = np.zeros(Nvar)
            grad[0:N_b] = grad_b_time
            grad[N_b+N_a:] = grad_u
            # Hessian assembly H_xx (Nvar x Nvar) sparse:
            # We'll assemble as block sparse: b-part tridiagonal from lower_b, diag_b, upper_b
            rows = []; cols = []; data = []
            # b-b block diag
            for i in range(N_b):
                if diag_b[i] != 0.0:
                    rows.append(i); cols.append(i); data.append(diag_b[i])
            for i in range(n):
                if lower_b[i] != 0.0:
                    rows.append(i); cols.append(i+1); data.append(lower_b[i])  # actually lower was H[i+1,i] but we'll place symmetric
                    rows.append(i+1); cols.append(i); data.append(lower_b[i])
            # u-u block: add H_u_blocks on respective positions (N_b+N_a offset)
            offset_u = N_b + N_a
            for i in range(n):
                H8 = H_u_blocks[i]
                for r_idx in range(8):
                    for c_idx in range(8):
                        val = H8[r_idx, c_idx]
                        if abs(val) > 0:
                            rows.append(offset_u + i*8 + r_idx)
                            cols.append(offset_u + i*8 + c_idx)
                            data.append(val)
            H_csr = sp.csr_matrix((data, (rows, cols)), shape=(Nvar, Nvar))
            # equality residuals and A
            A_eq, r_eq = equality_residuals_and_A(x)
            # compute combined gradient to feed into KKT: grad (already)
            # Solve KKT system for Newton step
            try:
                dx, dy = assemble_kkt_and_solve(H_csr, A_eq, grad, r_eq)
            except Exception as e:
                raise RuntimeError("KKT solve failed: " + str(e))
            # line search to maintain feasibility for barrier (ensure Fmax - ||u|| > 0)
            alpha = 1.0
            # backtracking line search parameters
            beta = 0.5
            # ensure b stays positive (for sqrt) and u stays inside friction (slack>0)
            for ls_it in range(30):
                x_new = x + alpha * dx
                b_new = x_new[0:N_b]
                u_new = x_new[N_b+N_a:].reshape((n,8))
                # check b positivity
                if np.any(b_new <= 0):
                    alpha *= beta
                    continue
                # check friction slack
                bad = False
                for i in range(n):
                    for w in range(4):
                        normuw = np.linalg.norm(u_new[i,2*w:2*w+2])
                        if normuw >= Fmax_per_wheel:
                            bad = True; break
                    if bad: break
                if bad:
                    alpha *= beta
                    continue
                # accept
                break
            x = x + alpha * dx
            # convergence check (norm of KKT residual)
            kkt_norm = np.linalg.norm(np.concatenate([grad, r_eq]))
            if kkt_norm < 1e-6:
                break
        # increase tau (sharpen barrier)
        tau *= tau_multiplier

    # unpack x
    b_final = x[0:N_b]
    a_final = x[N_b:N_b+N_a]
    u_final = x[N_b+N_a:].reshape((n,8))
    return b_final, a_final, u_final

# ----------------------
# Example usage
if __name__ == "__main__":
    # simple semicircle path points
    n = 40
    theta = np.linspace(0,1,n+1)
    R = 5.0
    angles = np.pi * theta
    s = np.vstack([ R*np.cos(angles), R*np.sin(angles) ]).T
    v0 = 0.5
    # wheel positions in body frame: [x_forward, y_right] (FL, FR, RL, RR)
    # simple car dimensions
    lf = 0.2  # front axle to CG
    lr = -0.2 # rear axle to CG (negative backwards)
    half_track = 0.1
    wheel_pos = np.array([
        [ lf,  half_track],
        [ lf, -half_track],
        [ lr,  half_track],
        [ lr, -half_track]
    ])
    def steering(i):
        # small steering function proportional to curvature of semicircle (simple)
        return 0.0  # keep straight for simplicity; user can set nonzero
    b, a, u = solve_banded_kkt_ackermann(s, v0, n, wheel_pos, vehicle_mass=2.0, mu=0.8, steering_delta_fun=steering)
    print("Solved b (first 8):", b[:8])
    print("Solved a (first 6):", a[:6])
    print("Solved u (first midpoint):", u[0])
