"""
speedopt_numba.py

Full-KKT + barrier interior-point minimum-time speed optimizer
for a path (raceline), Ackermann steering 4WD friction-circle model,
with numba JIT acceleration for hot loops.

Usage:
  - Edit PARAMETERS block below for vehicle params and raceline input
  - Run: python speedopt_numba_solver.py
  - Output: prints runtime and returns arrays (x,y,kappa,v) and plots speed profile.

Dependencies:
  pip install numpy scipy matplotlib numba
"""

"""
speedopt_numba_solver.py — integrated safe version
"""
import os
import numpy as np
import pandas as pd
import scipy.sparse as sp
import scipy.sparse.linalg as spla
import matplotlib.pyplot as plt
from time import perf_counter

# Try import numba
try:
    import numba as nb
    NUMBA_AVAILABLE = True
except Exception:
    NUMBA_AVAILABLE = False

# ----- User Inputs -----
# /home/dawgs_nx/f1tenth_dawgs/src/peripheral/racetracks/
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/racetracks/"
map_name = "levine/levine_blacked_"

csv_path = os.path.join(map_dir, map_name + "frenet_track.csv")
out_path = os.path.join(map_dir, map_name + "numba_speedopted.csv")

# ---------------------------
# PARAMETERS
vehicle_mass = 4.3
wheel_base = 0.33
mu = 0.9
g = 9.81
initial_speed = 0.0
tau0 = 0.3
tau_multiplier = 9.0
max_newton_iters = 25
wheel_positions_body = np.array([
    [ 0.18,  0.15],
    [ 0.18, -0.15],
    [-0.18,  0.15],
    [-0.18, -0.15]
])
# ---------------------------

# 2. Load path with preprocessing
def load_path(path, eps=1e-8):
    df = pd.read_csv(path)
    if {'x','y'}.issubset(df.columns):
        coords = df[['x','y']].to_numpy(dtype=float)
    else:
        coords = df.iloc[:, :2].to_numpy(dtype=float)

    # Remove near-duplicate nodes
    if coords.shape[0] >= 2:
        diffs = np.diff(coords, axis=0)
        keep = np.hypot(diffs[:,0], diffs[:,1]) > eps
        coords = np.vstack([coords[0], coords[1:][keep]])

    # Ensure at least 2 nodes
    if coords.shape[0] < 2:
        raise ValueError("Raceline must have at least 2 valid points after cleaning.")
    return coords

# ------------- numba helpers -------------
if NUMBA_AVAILABLE:
    njit = nb.njit
else:
    def njit(func=None, **kwargs):
        def wrap(f): return f
        if func is None: return wrap
        return func

@njit(cache=True, fastmath=True)
def normalize_nb(vx, vy):
    n = (vx*vx + vy*vy)**0.5
    if n == 0.0:
        return 0.0, 0.0, 0.0
    return vx/n, vy/n, n

@njit(cache=True, fastmath=True)
def rotation_apply(theta, vecx, vecy):
    c = np.cos(theta); s = np.sin(theta)
    rx = c*vecx - s*vecy
    ry = s*vecx + c*vecy
    return rx, ry

@njit(cache=True, fastmath=True)
def Rtilde_ackermann_nb(tanx, tany, steering_delta, yaw, outR):
    # tangent unit
    tx, ty, _ = normalize_nb(tanx, tany)
    nx = -ty; ny = tx  # normal
    for i in range(4):
        delta = steering_delta if i < 2 else 0.0
        rot = yaw + delta
        fx_x, fx_y = rotation_apply(rot, 1.0, 0.0)
        fy_x, fy_y = rotation_apply(rot, 0.0, 1.0)
        outR[0*8 + 2*i + 0] = tx*fx_x + ty*fx_y
        outR[1*8 + 2*i + 0] = nx*fx_x + ny*fx_y
        outR[0*8 + 2*i + 1] = tx*fy_x + ty*fy_y
        outR[1*8 + 2*i + 1] = nx*fy_x + ny*fy_y
    return

@njit(cache=True, fastmath=True)
def barrier_u_phi_grad_hess_nb(u8, Fmax, tau, grad_out, H_out):
    phi = 0.0
    for i in range(8): grad_out[i] = 0.0
    for i in range(64): H_out[i] = 0.0
    for w in range(4):
        ux = u8[2*w]; uy = u8[2*w+1]
        normu = (ux*ux + uy*uy)**0.5 + 1e-12
        slack = Fmax - normu
        if slack <= 0.0:
            phi += 1e12
            continue
        phi += -tau * np.log(slack)
        gfac = tau / (normu * slack)
        grad_out[2*w]   += gfac * ux
        grad_out[2*w+1] += gfac * uy
        inv_norm = 1.0 / normu
        inv_norm3 = inv_norm / (normu*normu)
        uu00 = ux*ux; uu01 = ux*uy; uu11 = uy*uy
        t11 = (inv_norm - uu00 * inv_norm3) / slack
        t12 = (-uu01 * inv_norm3) / slack
        t22 = (inv_norm - uu11 * inv_norm3) / slack
        fac2 = (inv_norm*inv_norm) / (slack*slack)
        h00 = tau * (t11 + uu00 * fac2)
        h01 = tau * (t12 + uu01 * fac2)
        h11 = tau * (t22 + uu11 * fac2)
        idx = 2*w
        H_out[(idx+0)*8 + (idx+0)] = h00
        H_out[(idx+0)*8 + (idx+1)] = h01
        H_out[(idx+1)*8 + (idx+0)] = h01
        H_out[(idx+1)*8 + (idx+1)] = h11
    return phi

@njit(cache=True, fastmath=True)
def time_obj_and_derivs_nb(b, dtheta, grad_out, lower_out, diag_out, upper_out):
    n1 = b.shape[0]
    n = n1 - 1
    for i in range(n1):
        grad_out[i] = 0.0
        diag_out[i] = 0.0
    for i in range(n):
        lower_out[i] = 0.0
        upper_out[i] = 0.0
    obj = 0.0
    for i in range(n):
        si = b[i]**0.5
        sj = b[i+1]**0.5
        denom = si + sj + 1e-20
        obj += 2.0 * dtheta / denom
        d_bi = - dtheta / ((denom**2) * si + 1e-20)
        d_bj = - dtheta / ((denom**2) * sj + 1e-20)
        grad_out[i]   += d_bi
        grad_out[i+1] += d_bj
        H_ii = dtheta / (denom**3 * (si**2 + 1e-20)) + dtheta / (2.0 * denom**2 * (si**3 + 1e-20))
        H_jj = dtheta / (denom**3 * (sj**2 + 1e-20)) + dtheta / (2.0 * denom**2 * (sj**3 + 1e-20))
        H_ij = dtheta * (1.0/(denom**3 * si * sj + 1e-20))
        diag_out[i]   += H_ii
        diag_out[i+1] += H_jj
        lower_out[i]  += H_ij
        upper_out[i]  += H_ij
    return obj

# -----------------------
# Path utilities
# -----------------------
def compute_curvature(xs, ys):
    dx = np.gradient(xs)
    dy = np.gradient(ys)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = (dx*dx + dy*dy)**1.5 + 1e-12
    kappa = (dx * ddy - dy * ddx) / denom
    # sanitize
    kappa = np.where(np.isfinite(kappa), kappa, 0.0)
    return kappa

# -----------------------
# Main solver (full KKT + barrier)
# -----------------------
def solve_full_kkt_with_numba(s_nodes,
                              initial_speed,
                              vehicle_mass, mu, g,
                              wheel_positions_body,
                              steering_delta_fun,
                              max_newton_iters=20, tau0=0.5, tau_multiplier=8.0,
                              reg_H=1e-8):
    m = s_nodes.shape[0]
    n = m - 1
    dtheta = 1.0 / n

    # midpoints tangent approx
    s_prime = (s_nodes[1:] - s_nodes[:-1]) / dtheta  # (n,2)

    # variable sizes
    N_b = n + 1
    N_a = 2 * n  # a_t, a_n per midpoint
    N_u = n * 8
    Nvar = N_b + N_a + N_u

    # indexing helpers
    def idx_a_t(i): return N_b + 2*i
    def idx_a_n(i): return N_b + 2*i + 1

    # initial x
    seg_len0 = max(np.linalg.norm(s_nodes[1] - s_nodes[0]), 1e-6)
    b0_val = (abs(initial_speed) * dtheta / seg_len0)**2
    b_init = max(b0_val*0.5, 1e-6)
    b = np.ones(N_b) * b_init
    b[0] = max(b0_val, 1e-6)
    a = np.zeros(N_a)
    u = np.zeros(N_u)
    x = np.concatenate([b, a, u])

    Fmax_per_wheel = mu * vehicle_mass * g / 4.0

    # buffers
    grad_b = np.zeros(N_b)
    lower_b = np.zeros(n)
    diag_b = np.zeros(N_b)
    upper_b = np.zeros(n)

    outR = np.zeros(16)
    grad_u_block = np.zeros(8)
    H_u_block = np.zeros(64)

    tau = tau0
    for barrier_round in range(4):
        for it_newton in range(max_newton_iters):
            # objective (b-block)
            _ = time_obj_and_derivs_nb(x[0:N_b], dtheta, grad_b, lower_b, diag_b, upper_b)

            # gradient
            grad = np.zeros(Nvar)
            grad[0:N_b] = grad_b

            # Hessian entries
            H_rows=[]; H_cols=[]; H_data=[]
            for i in range(N_b):
                v = diag_b[i]
                if v != 0.0:
                    H_rows.append(i); H_cols.append(i); H_data.append(v)
            for i in range(n):
                v = lower_b[i]
                if v != 0.0:
                    H_rows.append(i); H_cols.append(i+1); H_data.append(v)
                    H_rows.append(i+1); H_cols.append(i); H_data.append(v)

            # u-block barrier
            obj_bar = 0.0
            for i in range(n):
                u_i = x[N_b+N_a + i*8 : N_b+N_a + (i+1)*8]
                phi = barrier_u_phi_grad_hess_nb(u_i, Fmax_per_wheel, tau, grad_u_block, H_u_block)
                obj_bar += phi
                grad[N_b+N_a + i*8 : N_b+N_a + (i+1)*8] = grad_u_block

                base = N_b+N_a + i*8
                for rr in range(8):
                    row = base + rr
                    for cc in range(8):
                        v = H_u_block[rr*8 + cc]
                        if v != 0.0:
                            H_rows.append(row)
                            H_cols.append(base + cc)
                            H_data.append(v)

            # check H_data finite
            if len(H_data) and (not np.isfinite(H_data).all()):
                bad_idx = np.where(~np.isfinite(H_data))[0][0]
                raise ValueError(f"H contains NaN/Inf at entry {bad_idx}")

            H = sp.csr_matrix((H_data, (H_rows, H_cols)), shape=(Nvar, Nvar))
            if reg_H > 0:
                H = H + reg_H * sp.eye(Nvar, format='csr')

            # equality constraints
            rowsA=[]; colsA=[]; dataA=[]; res=[]
            eq_idx = 0

            # To avoid tan=(0,0) degeneracy, build a safe tangent if needed
            # use s_prime; if both 0, try neighboring direction or default x-axis
            def safe_tangent(i):
                tx, ty = s_prime[i,0], s_prime[i,1]
                if abs(tx) + abs(ty) > 0.0:
                    return tx, ty
                # fallback: look neighbors
                if i > 0:
                    tpx, tpy = s_prime[i-1,0], s_prime[i-1,1]
                    if abs(tpx)+abs(tpy) > 0.0:
                        return tpx, tpy
                if i+1 < n:
                    tnx, tny = s_prime[i+1,0], s_prime[i+1,1]
                    if abs(tnx)+abs(tny) > 0.0:
                        return tnx, tny
                return 1.0, 0.0  # final fallback

            for i in range(n):
                tanx, tany = safe_tangent(i)
                yaw = np.arctan2(tany, tanx)
                steering = steering_delta_fun(i)
                if not np.isfinite(steering):
                    steering = 0.0

                outR.fill(0.0)
                Rtilde_ackermann_nb(tanx, tany, steering, yaw, outR)

                # quick sanity
                if not np.isfinite(outR).all():
                    raise ValueError(f"outR has NaN/Inf at midpoint {i}")

                # tangent row
                prow = 0
                base_u = N_b+N_a + i*8
                for jcol in range(8):
                    rowsA.append(eq_idx + prow); colsA.append(base_u + jcol)
                    dataA.append(outR[prow*8 + jcol])
                rowsA.append(eq_idx + prow); colsA.append(idx_a_t(i)); dataA.append(-vehicle_mass)
                left0 = 0.0
                for jcol in range(8):
                    left0 += outR[0*8 + jcol] * x[base_u + jcol]
                right0 = vehicle_mass * x[idx_a_t(i)]
                res.append(left0 - right0)

                # normal row
                prow = 1
                for jcol in range(8):
                    rowsA.append(eq_idx + prow); colsA.append(base_u + jcol)
                    dataA.append(outR[prow*8 + jcol])
                rowsA.append(eq_idx + prow); colsA.append(idx_a_n(i)); dataA.append(-vehicle_mass)
                left1 = 0.0
                for jcol in range(8):
                    left1 += outR[1*8 + jcol] * x[base_u + jcol]
                right1 = vehicle_mass * x[idx_a_n(i)]
                res.append(left1 - right1)

                eq_idx += 2

            # b–a_t relation
            for i in range(1, n+1):
                rowsA.append(eq_idx); colsA.append(i); dataA.append(1.0)
                rowsA.append(eq_idx); colsA.append(i-1); dataA.append(-1.0)
                rowsA.append(eq_idx); colsA.append(idx_a_t(i-1)); dataA.append(-2.0 * dtheta)
                res.append(x[i] - x[i-1] - 2.0 * x[idx_a_t(i-1)] * dtheta)
                eq_idx += 1

            # b0 fixed
            rowsA.append(eq_idx); colsA.append(0); dataA.append(1.0)
            # residual is (b0 - b0_val)
            seg_len0 = max(np.linalg.norm(s_nodes[1] - s_nodes[0]), 1e-6)
            b0_val = (abs(initial_speed) * dtheta / seg_len0)**2
            res.append(x[0] - b0_val)
            eq_idx += 1

            dataA = np.asarray(dataA, dtype=float)
            if len(dataA) and (not np.isfinite(dataA).all()):
                bad_idx = np.where(~np.isfinite(dataA))[0][0]
                raise ValueError(f"A_eq data has NaN/Inf at entry {bad_idx}")

            A_eq = sp.csr_matrix((dataA, (rowsA, colsA)), shape=(eq_idx, Nvar))
            r_eq = np.asarray(res, dtype=float)
            if not np.isfinite(r_eq).all():
                bad_idx = np.where(~np.isfinite(r_eq))[0][0]
                raise ValueError(f"r_eq has NaN/Inf at entry {bad_idx}")

            # KKT
            top = sp.hstack([H, A_eq.transpose()]).tocsr()
            bottom = sp.hstack([A_eq, sp.csr_matrix((A_eq.shape[0], A_eq.shape[0]))]).tocsr()
            K = sp.vstack([top, bottom]).tocsr()
            rhs = -np.concatenate([grad, r_eq])

            if np.isnan(K.data).any() or np.isinf(K.data).any():
                # Pinpoint which block caused it
                msg = []
                if len(H.data) and (np.isnan(H.data).any() or np.isinf(H.data).any()):
                    msg.append("H block has NaN/Inf")
                if len(A_eq.data) and (np.isnan(A_eq.data).any() or np.isinf(A_eq.data).any()):
                    msg.append("A_eq block has NaN/Inf")
                raise ValueError("K matrix contains NaN or Inf values: " + ", ".join(msg))

            # solve
            try:
                lu = spla.splu(K.tocsc())
            except RuntimeError:
                Hreg = H + (10.0 * reg_H) * sp.eye(Nvar, format='csr')
                top = sp.hstack([Hreg, A_eq.transpose()]).tocsr()
                K = sp.vstack([top, bottom]).tocsr()
                lu = spla.splu(K.tocsc())

            sol = lu.solve(rhs)
            dx = sol[:Nvar]
            # dy = sol[Nvar:]  # unused here

            # line search for feasibility
            alpha = 1.0
            for _ in range(25):
                x_new = x + alpha * dx
                # positivity of b
                if np.any(x_new[0:N_b] <= 0) or (not np.isfinite(x_new[0:N_b]).all()):
                    alpha *= 0.5; continue
                # friction circle feasibility
                bad = False
                for i in range(n):
                    base = N_b + N_a + i*8
                    for w in range(4):
                        ux, uy = x_new[base + 2*w : base + 2*w + 2]
                        if not np.isfinite(ux) or not np.isfinite(uy):
                            bad = True; break
                        if np.hypot(ux, uy) >= Fmax_per_wheel:
                            bad = True; break
                    if bad: break
                if bad:
                    alpha *= 0.5; continue
                break
            x = x + alpha * dx

            # recompute KKT residual norm after update
            _ = time_obj_and_derivs_nb(x[0:N_b], dtheta, grad_b, lower_b, diag_b, upper_b)
            grad_check = np.zeros_like(grad)
            grad_check[0:N_b] = grad_b
            r_eq_check = A_eq @ x
            kkt_norm = np.linalg.norm(np.concatenate([grad_check, r_eq_check]))
            if not np.isfinite(kkt_norm):
                raise ValueError("KKT norm became NaN/Inf")
            if kkt_norm < 1e-6:
                break

        tau *= tau_multiplier

    b_final = x[0:N_b]
    a_final = x[N_b:N_b+N_a]
    u_final = x[N_b+N_a:].reshape((n,8))
    return b_final, a_final, u_final

# -----------------------
# High-level wrapper
# -----------------------
def optimize_raceline_to_speed(raceline_xy,
                               vehicle_mass, mu, g,
                               wheelbase, initial_speed,
                               wheel_positions_body):
    m = raceline_xy.shape[0]
    if m < 2:
        raise ValueError("raceline must have at least 2 points")
    n = m - 1

    xs = raceline_xy[:,0]; ys = raceline_xy[:,1]
    kappa_nodes = compute_curvature(xs, ys)  # length m

    # safe steering
    steering_mid = np.arctan(wheelbase * kappa_nodes[:-1])
    steering_mid[~np.isfinite(steering_mid)] = 0.0

    def steering_fun(i):
        return float(steering_mid[i])

    print("steering min/max:", float(np.min(steering_mid)), float(np.max(steering_mid)))
    print("kappa   min/max:", float(np.min(kappa_nodes)), float(np.max(kappa_nodes)))

    b, a, u = solve_full_kkt_with_numba(
        raceline_xy, initial_speed,
        vehicle_mass, mu, g,
        wheel_positions_body,
        steering_fun,
        max_newton_iters=max_newton_iters,
        tau0=tau0, tau_multiplier=tau_multiplier
    )

    dtheta = 1.0 / (m-1)
    seg_lengths = np.linalg.norm(raceline_xy[1:] - raceline_xy[:-1], axis=1)
    v_theta = np.sqrt(np.maximum(b, 0.0))
    seg_len_nodes = np.zeros_like(b)
    seg_len_nodes[:-1] = seg_lengths
    seg_len_nodes[-1]  = seg_lengths[-1]
    v_linear_nodes = v_theta * seg_len_nodes / dtheta

    return {
        "x": raceline_xy[:,0],
        "y": raceline_xy[:,1],
        "kappa": kappa_nodes,
        "v": v_linear_nodes
    }

# -----------------------
# Demo / run
# -----------------------
if __name__ == "__main__":
    raceline = load_path(csv_path)
    t0 = perf_counter()
    result = optimize_raceline_to_speed(
        raceline,
        vehicle_mass=vehicle_mass, mu=mu, g=g,
        wheelbase=wheel_base, initial_speed=initial_speed,
        wheel_positions_body=wheel_positions_body
    )
    t1 = perf_counter()

    df_out = pd.DataFrame({
        'x': result['x'],
        'y': result['y'],
        'v': result['v'],
        'kappa': result['kappa']
    })
    df_out.round({
        'x': 3,
        'y': 3,
        'kappa': 4,
        'v': 3
    }).to_csv(out_path, index=False)

    print(f"Optimization finished in {t1-t0:.3f} s; v mean: {df_out['v'].mean():.2f}")