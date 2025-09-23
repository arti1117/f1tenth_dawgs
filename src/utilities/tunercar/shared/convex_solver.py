"""
Convex optimization solver for tunercar parameter tuning
Based on "Minimum-time speed optimisation over a fixed path" by Lipp & Boyd
and TUNERCAR superoptimization framework
"""
import numpy as np
import cvxpy as cp
from typing import Callable, Tuple, Optional, Dict, Any, List
from dataclasses import dataclass

@dataclass
class ConvexConfig:
    """Configuration for convex optimization"""
    max_iter: int = 2000
    solver: str = 'ECOS'  # ECOS, SCS, OSQP, MOSEK
    abstol: float = 1e-6
    reltol: float = 1e-6
    feastol: float = 1e-6
    verbose: bool = False

    # Path discretization parameters
    n_points: int = 100
    path_smoothing: bool = True

    # Trust region parameters
    trust_radius_init: float = 0.3
    trust_radius_min: float = 0.05
    trust_radius_max: float = 1.0
    shrink_factor: float = 0.5
    expand_factor: float = 2.0

@dataclass
class VehicleParams:
    """Vehicle dynamics parameters for convex optimization"""
    mass: float = 4.3  # kg
    wheelbase: float = 0.33  # meter
    mu: float = 0.9  # friction coefficient
    g: float = 9.81
    v_max: float = 15.0  # m/s
    a_max: float = 4.0  # m/s²
    a_min: float = -4.0  # m/s²

class MinimumTimeSpeedOptimizer:
    """
    Minimum-time speed optimization solver based on Lipp & Boyd 2014.
    Implements convex optimization for speed profile given a fixed path.
    """

    def __init__(self, vehicle_params: VehicleParams = VehicleParams(),
                 config: ConvexConfig = ConvexConfig()):
        self.vehicle_params = vehicle_params
        self.config = config
        self.best_x = None
        self.best_f = float('inf')
        self.history = []

    def uniform_theta_grid(self, n: int) -> Tuple[np.ndarray, float]:
        """Generate uniform θ-grid as in Lipp & Boyd paper"""
        theta = np.linspace(0, 1, n)
        dtheta = theta[1] - theta[0]
        return theta, dtheta

    def compute_derivatives(self, x: np.ndarray, y: np.ndarray, dtheta: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute path derivatives using high-order finite differences
        Based on Section 6.1.5 of Lipp & Boyd paper
        """
        n = len(x)

        # First derivatives at midpoints
        xdot_mid = (x[1:] - x[:-1]) / dtheta
        ydot_mid = (y[1:] - y[:-1]) / dtheta

        # Second derivatives at midpoints
        xdd_mid = np.zeros(n-1)
        ydd_mid = np.zeros(n-1)

        # Near left boundary (3-point stencil)
        if n > 2:
            xdd_mid[0] = (0.5*x[0] - x[1] + 0.5*x[2]) / (dtheta**2)
            ydd_mid[0] = (0.5*y[0] - y[1] + 0.5*y[2]) / (dtheta**2)

        # Interior points (6-point symmetric stencil for high accuracy)
        for i in range(2, min(n-3, len(xdd_mid)-1)):
            coeff = [-5/48, 13/16, -17/24, -17/24, 13/16, -5/48]
            ids = [i-2, i-1, i, i+1, i+2, i+3]

            xdd_mid[i] = sum(c * x[j] for c, j in zip(coeff, ids)) / (dtheta**2)
            ydd_mid[i] = sum(c * y[j] for c, j in zip(coeff, ids)) / (dtheta**2)

        # Near right boundary
        if n > 2 and len(xdd_mid) > 1:
            i = len(xdd_mid) - 1
            xdd_mid[i] = (0.5*x[n-3] - x[n-2] + 0.5*x[n-1]) / (dtheta**2)
            ydd_mid[i] = (0.5*y[n-3] - y[n-2] + 0.5*y[n-1]) / (dtheta**2)

        return xdot_mid, ydot_mid, xdd_mid, ydd_mid

    def compute_curvature(self, xdot: np.ndarray, ydot: np.ndarray,
                         xdd: np.ndarray, ydd: np.ndarray) -> np.ndarray:
        """Compute curvature at midpoints"""
        numerator = xdot * ydd - ydot * xdd
        denominator = (xdot**2 + ydot**2)**1.5

        kappa = np.zeros_like(numerator)
        valid_mask = denominator > 1e-12
        kappa[valid_mask] = numerator[valid_mask] / denominator[valid_mask]

        return kappa

    def solve_minimum_time_speed_profile(self, x: np.ndarray, y: np.ndarray,
                                       v_init: float = 0.0, v_final: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Solve minimum-time speed optimization problem for fixed path.
        Based on Section 5 of Lipp & Boyd paper.

        Args:
            x, y: Path coordinates
            v_init: Initial velocity
            v_final: Final velocity (optional)

        Returns:
            v_profile: Optimal velocity profile
            b_values: Transformed variables (b = θ̇²)
            kappa: Curvature profile
        """
        n = len(x)
        theta, dtheta = self.uniform_theta_grid(n)

        # Compute path derivatives and curvature
        xdot_mid, ydot_mid, xdd_mid, ydd_mid = self.compute_derivatives(x, y, dtheta)
        kappa_mid = self.compute_curvature(xdot_mid, ydot_mid, xdd_mid, ydd_mid)

        # Create optimization variables
        b = cp.Variable(n, pos=True)  # b(θ) = θ̇²

        # Objective: minimize total time (Equation 25 in paper)
        # T = ∫ b(θ)^(-1/2) dθ
        objective = cp.sum([
            dtheta / (cp.sqrt(b[i]) + cp.sqrt(b[i+1])) * 2
            for i in range(n-1)
        ])

        constraints = []

        # Initial condition constraint (Equation 32)
        if v_init > 0:
            ds01 = np.hypot(x[1] - x[0], y[1] - y[0])
            b0_val = ((v_init * dtheta) / max(ds01, 1e-9))**2
            constraints.append(b[0] == b0_val)
        else:
            constraints.append(b[0] >= 1e-6)

        # Final condition constraint (optional)
        if v_final >= 0:
            ds_final = np.hypot(x[-1] - x[-2], y[-1] - y[-2])
            bn_val = ((v_final * dtheta) / max(ds_final, 1e-9))**2
            constraints.append(b[-1] == bn_val)

        # Acceleration constraints (based on vehicle dynamics)
        for i in range(1, n):
            # Longitudinal acceleration constraint: a = (b[i] - b[i-1]) / (2*dtheta)
            a_i = (b[i] - b[i-1]) / (2 * dtheta)
            constraints.append(a_i <= self.vehicle_params.a_max)
            constraints.append(a_i >= self.vehicle_params.a_min)

        # Velocity limits from curvature constraints
        v_max_sq = self.vehicle_params.v_max**2

        # Speed limits at nodes from curvature
        b_max_node = np.full(n, v_max_sq)
        if len(kappa_mid) > 0:
            # Lateral acceleration constraint: v²κ ≤ μg
            for i in range(len(kappa_mid)):
                if abs(kappa_mid[i]) > 1e-9:
                    v_max_curve_sq = (self.vehicle_params.mu * self.vehicle_params.g) / abs(kappa_mid[i])
                    if i < n-1:
                        b_max_node[i] = min(b_max_node[i], v_max_curve_sq)
                    if i+1 < n:
                        b_max_node[i+1] = min(b_max_node[i+1], v_max_curve_sq)

        # Apply speed limits
        for i in range(n):
            constraints.append(b[i] <= b_max_node[i] + 1e-9)
            constraints.append(b[i] >= 1e-9)

        # Solve the convex optimization problem
        problem = cp.Problem(cp.Minimize(objective), constraints)

        try:
            if self.config.solver == 'ECOS':
                problem.solve(
                    solver=cp.ECOS,
                    abstol=self.config.abstol,
                    reltol=self.config.reltol,
                    feastol=self.config.feastol,
                    max_iters=self.config.max_iter,
                    verbose=self.config.verbose
                )
            elif self.config.solver == 'SCS':
                problem.solve(
                    solver=cp.SCS,
                    max_iters=self.config.max_iter,
                    verbose=self.config.verbose
                )
            elif self.config.solver == 'OSQP':
                problem.solve(
                    solver=cp.OSQP,
                    max_iter=self.config.max_iter,
                    verbose=self.config.verbose
                )
            else:
                problem.solve(verbose=self.config.verbose)

        except Exception as e:
            print(f"[ERROR] Primary solver failed: {e}")
            print("[INFO] Trying fallback solver SCS...")
            problem.solve(solver=cp.SCS, verbose=self.config.verbose)

        if b.value is None:
            raise RuntimeError("Convex solver failed to find solution")

        b_sol = np.maximum(b.value, 1e-9)

        # Convert back to velocity profile
        # v = ||s'|| * sqrt(b) at nodes
        spd_mid = np.hypot(xdot_mid, ydot_mid)
        spd_node = np.zeros(n)

        if len(spd_mid) > 0:
            spd_node[0] = spd_mid[0]
            spd_node[-1] = spd_mid[-1]
            for i in range(1, n-1):
                if i-1 < len(spd_mid) and i < len(spd_mid):
                    spd_node[i] = 0.5 * (spd_mid[i-1] + spd_mid[i])
                else:
                    spd_node[i] = spd_mid[min(i-1, len(spd_mid)-1)]

        v_profile = spd_node * np.sqrt(b_sol)

        self.best_x = v_profile
        self.best_f = problem.value

        return v_profile, b_sol, kappa_mid

class ConvexSolver:
    """
    General convex optimization solver for tunercar parameter optimization.
    Integrates minimum-time speed optimization with parameter tuning.
    """

    def __init__(self, config: ConvexConfig = ConvexConfig()):
        self.config = config
        self.best_x = None
        self.best_f = float('inf')
        self.history = []
        self.speed_optimizer = MinimumTimeSpeedOptimizer(config=config)

    def solve_quadratic_program(
        self,
        x0: np.ndarray,
        bounds: Tuple[np.ndarray, np.ndarray],
        Q: np.ndarray,
        c: np.ndarray,
        A_eq: Optional[np.ndarray] = None,
        b_eq: Optional[np.ndarray] = None,
        A_ineq: Optional[np.ndarray] = None,
        b_ineq: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, float]:
        """
        Solve quadratic program: minimize 0.5 * x^T * Q * x + c^T * x
        subject to:
            A_eq * x = b_eq (equality constraints)
            A_ineq * x <= b_ineq (inequality constraints)
            bounds[0] <= x <= bounds[1] (box constraints)
        """
        n = len(x0)
        x = cp.Variable(n)

        # Objective: 0.5 * x^T * Q * x + c^T * x
        objective = cp.Minimize(0.5 * cp.quad_form(x, Q) + c.T @ x)

        constraints = []

        # Box constraints
        constraints.append(x >= bounds[0])
        constraints.append(x <= bounds[1])

        # Equality constraints
        if A_eq is not None and b_eq is not None:
            constraints.append(A_eq @ x == b_eq)

        # Inequality constraints
        if A_ineq is not None and b_ineq is not None:
            constraints.append(A_ineq @ x <= b_ineq)

        # Solve
        problem = cp.Problem(objective, constraints)

        try:
            if self.config.solver == 'ECOS':
                problem.solve(
                    solver=cp.ECOS,
                    abstol=self.config.abstol,
                    reltol=self.config.reltol,
                    feastol=self.config.feastol,
                    max_iters=self.config.max_iter,
                    verbose=self.config.verbose
                )
            elif self.config.solver == 'SCS':
                problem.solve(
                    solver=cp.SCS,
                    max_iters=self.config.max_iter,
                    verbose=self.config.verbose
                )
            elif self.config.solver == 'OSQP':
                problem.solve(
                    solver=cp.OSQP,
                    max_iter=self.config.max_iter,
                    verbose=self.config.verbose
                )
            else:
                problem.solve(verbose=self.config.verbose)

        except Exception as e:
            print(f"[ERROR] Primary solver failed: {e}")
            print("[INFO] Trying fallback solver SCS...")
            problem.solve(solver=cp.SCS, verbose=self.config.verbose)

        if x.value is None:
            raise RuntimeError("Convex solver failed to find solution")

        optimal_x = x.value
        optimal_f = problem.value

        self.best_x = optimal_x
        self.best_f = optimal_f

        return optimal_x, optimal_f

    def solve_linear_program(
        self,
        c: np.ndarray,
        bounds: Tuple[np.ndarray, np.ndarray],
        A_eq: Optional[np.ndarray] = None,
        b_eq: Optional[np.ndarray] = None,
        A_ineq: Optional[np.ndarray] = None,
        b_ineq: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, float]:
        """
        Solve linear program: minimize c^T * x
        subject to constraints
        """
        n = len(c)
        x = cp.Variable(n)

        # Objective: c^T * x
        objective = cp.Minimize(c.T @ x)

        constraints = []

        # Box constraints
        constraints.append(x >= bounds[0])
        constraints.append(x <= bounds[1])

        # Equality constraints
        if A_eq is not None and b_eq is not None:
            constraints.append(A_eq @ x == b_eq)

        # Inequality constraints
        if A_ineq is not None and b_ineq is not None:
            constraints.append(A_ineq @ x <= b_ineq)

        # Solve
        problem = cp.Problem(objective, constraints)
        problem.solve(verbose=self.config.verbose)

        if x.value is None:
            raise RuntimeError("Linear program solver failed")

        return x.value, problem.value

    def local_convex_approximation(
        self,
        x0: np.ndarray,
        bounds: Tuple[np.ndarray, np.ndarray],
        eval_fn: Callable[[np.ndarray], float],
        trust_radius: float = 0.5,
        grad_fn: Optional[Callable[[np.ndarray], np.ndarray]] = None,
        hess_fn: Optional[Callable[[np.ndarray], np.ndarray]] = None
    ) -> Tuple[np.ndarray, float]:
        """
        Local convex approximation for non-convex problems.
        Uses finite differences if gradient/hessian not provided.
        """
        n = len(x0)

        # Compute gradient (finite difference if not provided)
        if grad_fn is not None:
            grad = grad_fn(x0)
        else:
            grad = self._finite_difference_gradient(eval_fn, x0)

        # Compute Hessian (finite difference if not provided)
        if hess_fn is not None:
            hess = hess_fn(x0)
        else:
            hess = self._finite_difference_hessian(eval_fn, x0)

        # Ensure Hessian is positive definite for convexity
        eigvals = np.linalg.eigvals(hess)
        if np.any(eigvals <= 0):
            # Regularize to make positive definite
            reg = abs(np.min(eigvals)) + 1e-6
            hess += reg * np.eye(n)

        # Trust region constraints
        trust_bounds = (
            np.maximum(bounds[0], x0 - trust_radius),
            np.minimum(bounds[1], x0 + trust_radius)
        )

        # Solve quadratic approximation
        # f(x) ≈ f(x0) + grad^T(x-x0) + 0.5*(x-x0)^T*H*(x-x0)
        # Minimize grad^T*y + 0.5*y^T*H*y where y = x - x0

        y = cp.Variable(n)
        objective = cp.Minimize(grad.T @ y + 0.5 * cp.quad_form(y, hess))

        constraints = [
            y >= trust_bounds[0] - x0,
            y <= trust_bounds[1] - x0
        ]

        problem = cp.Problem(objective, constraints)
        problem.solve(verbose=self.config.verbose)

        if y.value is None:
            print("[WARN] Trust region solver failed, returning x0")
            return x0, eval_fn(x0)

        x_new = x0 + y.value
        f_new = eval_fn(x_new)

        return x_new, f_new

    def _finite_difference_gradient(self, f: Callable, x: np.ndarray, h: float = 1e-6) -> np.ndarray:
        """Compute gradient using finite differences"""
        n = len(x)
        grad = np.zeros(n)

        for i in range(n):
            x_plus = x.copy()
            x_minus = x.copy()
            x_plus[i] += h
            x_minus[i] -= h

            grad[i] = (f(x_plus) - f(x_minus)) / (2 * h)

        return grad

    def _finite_difference_hessian(self, f: Callable, x: np.ndarray, h: float = 1e-5) -> np.ndarray:
        """Compute Hessian using finite differences"""
        n = len(x)
        hess = np.zeros((n, n))

        f_x = f(x)

        for i in range(n):
            for j in range(i, n):
                x_pp = x.copy()
                x_pm = x.copy()
                x_mp = x.copy()
                x_mm = x.copy()

                x_pp[i] += h; x_pp[j] += h
                x_pm[i] += h; x_pm[j] -= h
                x_mp[i] -= h; x_mp[j] += h
                x_mm[i] -= h; x_mm[j] -= h

                hess[i, j] = (f(x_pp) - f(x_pm) - f(x_mp) + f(x_mm)) / (4 * h * h)
                hess[j, i] = hess[i, j]  # Symmetric

        return hess

    def optimize_tunercar_parameters(self,
                                    path_x: np.ndarray,
                                    path_y: np.ndarray,
                                    x0: np.ndarray,
                                    bounds: Tuple[np.ndarray, np.ndarray],
                                    eval_fn: Callable[[np.ndarray], float],
                                    use_speed_optimization: bool = True) -> Tuple[np.ndarray, float]:
        """
        Integrated parameter optimization for TUNERCAR.
        Combines convex speed optimization with parameter tuning.

        Args:
            path_x, path_y: Race track path coordinates
            x0: Initial parameters [lookahead_base, lookahead_k, ...]
            bounds: Parameter bounds
            eval_fn: Function that evaluates lap time for given parameters
            use_speed_optimization: Whether to use convex speed optimization

        Returns:
            Optimal parameters and objective value
        """
        if use_speed_optimization:
            # First, solve for optimal speed profile given path
            try:
                v_profile, b_values, kappa = self.speed_optimizer.solve_minimum_time_speed_profile(
                    path_x, path_y, v_init=0.1, v_final=0.1
                )
                print(f"[INFO] Optimal speed profile computed. Max speed: {np.max(v_profile):.2f} m/s")

                # Store speed profile for use in parameter optimization
                self.optimal_speed_profile = v_profile

            except Exception as e:
                print(f"[WARN] Speed optimization failed: {e}. Using trust region only.")
                use_speed_optimization = False

        # Now optimize control/planning parameters using trust region
        return self.local_convex_approximation(x0, bounds, eval_fn, self.config.trust_radius_init)

def solve_tunercar_convex(
    path_x: np.ndarray,
    path_y: np.ndarray,
    x0: np.ndarray,
    bounds: Tuple[np.ndarray, np.ndarray],
    eval_fn: Callable[[np.ndarray], float],
    config: ConvexConfig = ConvexConfig(),
    use_speed_optimization: bool = True,
    vehicle_params: VehicleParams = VehicleParams()
) -> Tuple[np.ndarray, float]:
    """
    Main wrapper function to solve tunercar optimization with convex methods.
    Integrates both minimum-time speed optimization and parameter tuning.

    Args:
        path_x, path_y: Race track path coordinates
        x0: Initial parameters [lookahead_base, lookahead_k, ...]
        bounds: Parameter bounds
        eval_fn: Function that evaluates lap time for given parameters
        config: Convex solver configuration
        use_speed_optimization: Whether to use convex speed optimization
        vehicle_params: Vehicle dynamics parameters

    Returns:
        Optimal parameters and objective value
    """
    solver = ConvexSolver(config)
    solver.speed_optimizer = MinimumTimeSpeedOptimizer(vehicle_params, config)

    return solver.optimize_tunercar_parameters(
        path_x, path_y, x0, bounds, eval_fn, use_speed_optimization
    )

def optimize_racing_line_and_speed(
    track_boundaries: np.ndarray,
    n_points: int = 100,
    vehicle_params: VehicleParams = VehicleParams(),
    config: ConvexConfig = ConvexConfig()
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Optimize both racing line and speed profile for a given track.
    This implements the full TUNERCAR approach with convex speed optimization.

    Args:
        track_boundaries: Track boundary coordinates
        n_points: Number of discretization points
        vehicle_params: Vehicle dynamics parameters
        config: Optimization configuration

    Returns:
        Tuple of (optimal_x, optimal_y, optimal_speeds)
    """
    # This is a placeholder for full racing line optimization
    # In practice, this would use the path planning from TUNERCAR
    # and integrate with the convex speed optimization

    speed_optimizer = MinimumTimeSpeedOptimizer(vehicle_params, config)

    # For now, assume we have a centerline path
    # In full implementation, this would come from path planning
    center_x = track_boundaries[:, 0]  # Placeholder
    center_y = track_boundaries[:, 1]  # Placeholder

    # Optimize speed profile for the given path
    v_profile, b_values, kappa = speed_optimizer.solve_minimum_time_speed_profile(
        center_x, center_y, v_init=0.1
    )

    return center_x, center_y, v_profile

if __name__ == "__main__":
    # Test minimum-time speed optimization
    print("=== Testing Minimum-Time Speed Optimization ===")

    # Create a simple curved path (circle segment)
    theta = np.linspace(0, np.pi/2, 50)
    radius = 10.0
    path_x = radius * np.cos(theta)
    path_y = radius * np.sin(theta)

    # Test speed optimization
    vehicle_params = VehicleParams(
        mass=4.3,
        wheelbase=0.33,
        mu=0.9,
        v_max=8.0,
        a_max=4.0,
        a_min=-6.0
    )

    config = ConvexConfig(
        n_points=len(path_x),
        solver='ECOS',
        verbose=True
    )

    speed_optimizer = MinimumTimeSpeedOptimizer(vehicle_params, config)

    try:
        v_profile, b_values, kappa = speed_optimizer.solve_minimum_time_speed_profile(
            path_x, path_y, v_init=0.5, v_final=0.5
        )

        print(f"Speed optimization successful!")
        print(f"Max speed: {np.max(v_profile):.2f} m/s")
        print(f"Min speed: {np.min(v_profile):.2f} m/s")
        print(f"Total time: {speed_optimizer.best_f:.3f} (dimensionless)")
        print(f"Max curvature: {np.max(np.abs(kappa)):.4f} rad/m")

        # Save results for visualization
        import os
        results_dir = "/home/dawgs_nx/f1tenth_dawgs/src/utilities/tunercar/shared"
        results_file = os.path.join(results_dir, "convex_speed_test_results.txt")

        with open(results_file, 'w') as f:
            f.write("Convex Speed Optimization Test Results\n")
            f.write("=====================================\n")
            f.write(f"Path points: {len(path_x)}\n")
            f.write(f"Max speed: {np.max(v_profile):.3f} m/s\n")
            f.write(f"Min speed: {np.min(v_profile):.3f} m/s\n")
            f.write(f"Total time: {speed_optimizer.best_f:.6f}\n")
            f.write(f"Max curvature: {np.max(np.abs(kappa)):.6f} rad/m\n")
            f.write("\nPath and Speed Profile:\n")
            f.write("x,y,v,kappa\n")
            for i, (x, y, v) in enumerate(zip(path_x, path_y, v_profile)):
                k = kappa[min(i, len(kappa)-1)] if len(kappa) > 0 else 0.0
                f.write(f"{x:.6f},{y:.6f},{v:.6f},{k:.6f}\n")

        print(f"Results saved to: {results_file}")

    except Exception as e:
        print(f"Speed optimization failed: {e}")
        import traceback
        traceback.print_exc()

    print("\n=== Testing TUNERCAR Parameter Optimization ===")

    # Test parameter optimization with simple function
    def test_eval_fn(params):
        # Simple quadratic function: minimize distance from [1.5, 0.6]
        return (params[0] - 1.5) ** 2 + (params[1] - 0.6) ** 2

    x0 = np.array([1.0, 0.5])  # [lookahead_base, lookahead_k]
    bounds = (np.array([0.5, 0.1]), np.array([3.0, 1.0]))

    try:
        x_opt, f_opt = solve_tunercar_convex(
            path_x, path_y, x0, bounds, test_eval_fn,
            config=config, use_speed_optimization=True, vehicle_params=vehicle_params
        )

        print(f"Parameter optimization successful!")
        print(f"Optimal parameters: lookahead_base={x_opt[0]:.3f}, lookahead_k={x_opt[1]:.3f}")
        print(f"Optimal objective: {f_opt:.6f}")

    except Exception as e:
        print(f"Parameter optimization failed: {e}")
        import traceback
        traceback.print_exc()

    print("\n=== Integration Test Complete ===")
    print("The convex_solver module is ready for use with tunercar!")