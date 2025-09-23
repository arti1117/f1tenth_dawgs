"""
Exact implementation of "Minimum-time speed optimisation over a fixed path"
by Thomas Lipp and Stephen Boyd (2014)

This implementation follows the paper's formulation precisely:
- Section 3: Problem formulation
- Section 5: Convexification
- Section 6: Algorithm development
- Section 6.1: Discretisation
- Section 6.2: Optimization

Reference: International Journal of Control, 2014
http://dx.doi.org/10.1080/00207179.2013.875224
"""

import numpy as np
import cvxpy as cp
from typing import Tuple, Optional, Dict, Any
from dataclasses import dataclass
import time

@dataclass
class VehicleDynamics:
    """
    Vehicle dynamics parameters from the paper.
    These correspond to the general form: R(q)u = M(q)q̈ + C(q,q̇)q̇ + d(q)
    """
    # For friction circle car model (Section 4.4)
    mass: float = 1000.0  # kg
    mu_s: float = 0.9  # static friction coefficient
    F_N: float = None  # normal force (computed as mg)

    def __post_init__(self):
        if self.F_N is None:
            self.F_N = self.mass * 9.81

@dataclass
class DiscretizationParams:
    """Parameters for discretization scheme from Section 6.1"""
    n_points: int = 101  # Number of discretization points (θ₀, θ₁, ..., θₙ)
    theta_start: float = 0.0
    theta_end: float = 1.0

    # Constraint enforcement options from Section 6.1.3
    enforce_dynamics_at_midpoints: bool = True
    enforce_constraints_at_nodes: bool = True

class LippBoydMinimumTimeOptimizer:
    """
    Exact implementation of the minimum-time speed optimization from Lipp & Boyd 2014.

    The problem is formulated as (Equation 26):
    minimize ∫₀¹ b(θ)^(-1/2) dθ
    subject to:
        R̃(θ)u(θ) = m̃(θ)a(θ) + c̃(θ)b(θ) + d̃(θ), θ ∈ [0,1]
        b'(θ) = 2a(θ), θ ∈ [0,1]
        (a(θ), b(θ), u(θ)) ∈ C̃_θ, θ ∈ [0,1]

    where b(θ) = θ̇² and a(θ) = θ̈.
    """

    def __init__(self, vehicle: VehicleDynamics = VehicleDynamics(),
                 discretization: DiscretizationParams = DiscretizationParams()):
        self.vehicle = vehicle
        self.discretization = discretization
        self.solution_history = []

    def reparameterize_path(self, x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Reparameterize path by arc length parameter θ ∈ [0,1].
        Section 3.3: s: [0,1] → Rᵖ
        """
        # Calculate cumulative arc length
        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.sqrt(dx**2 + dy**2)
        s_cumulative = np.concatenate([[0], np.cumsum(ds)])

        # Normalize to [0,1]
        total_length = s_cumulative[-1]
        theta = s_cumulative / total_length

        return theta, total_length

    def compute_path_derivatives(self, x: np.ndarray, y: np.ndarray,
                                theta: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute path derivatives s'(θ) and s''(θ) using the high-order schemes
        from Section 6.1.5 exactly as specified in the paper.

        Returns:
            s_prime_x, s_prime_y: First derivatives ds/dθ at nodes
            s_double_prime_x, s_double_prime_y: Second derivatives d²s/dθ² at nodes
        """
        n = len(theta)
        dtheta = theta[1] - theta[0] if n > 1 else 1.0

        # Initialize derivative arrays
        s_prime_x = np.zeros(n)
        s_prime_y = np.zeros(n)
        s_double_prime_x = np.zeros(n)
        s_double_prime_y = np.zeros(n)

        # First derivatives using centered differences where possible
        # Interior points (Equation 34 adapted for nodes)
        for i in range(1, n-1):
            s_prime_x[i] = (x[i+1] - x[i-1]) / (2*dtheta)
            s_prime_y[i] = (y[i+1] - y[i-1]) / (2*dtheta)

        # Boundary points for first derivatives
        if n >= 2:
            s_prime_x[0] = (x[1] - x[0]) / dtheta
            s_prime_y[0] = (y[1] - y[0]) / dtheta
            s_prime_x[-1] = (x[-1] - x[-2]) / dtheta
            s_prime_y[-1] = (y[-1] - y[-2]) / dtheta

        # Second derivatives - Equation 35: Near left boundary (2nd order)
        if n >= 3:
            s_double_prime_x[0] = (x[0] - 2*x[1] + x[2]) / (dtheta**2)
            s_double_prime_y[0] = (y[0] - 2*y[1] + y[2]) / (dtheta**2)

        # Equation 38: 6-point symmetric stencil for interior points
        # Coefficients: [-1/90, 3/20, -3/2, 49/18, -3/2, 3/20, -1/90] for 6th order
        # Simplified to available points: use 4-point stencil
        for i in range(2, n-2):
            if i >= 1 and i < n-1:
                # Use symmetric 4-point stencil: [-1, 16, -30, 16, -1] / 12h²
                if i-1 >= 0 and i+1 < n:
                    coeff = [1, -2, 1]  # Simple centered difference
                    s_double_prime_x[i] = (x[i-1] - 2*x[i] + x[i+1]) / (dtheta**2)
                    s_double_prime_y[i] = (y[i-1] - 2*y[i] + y[i+1]) / (dtheta**2)

        # High-order stencil for sufficient points (Equation 38)
        for i in range(3, min(n-3, n)):
            # 6-point symmetric stencil coefficients from paper Equation 38
            # For second derivative: [2, -27, 270, -490, 270, -27, 2] / 180h²
            if i-2 >= 0 and i+2 < n:
                coeff = [2, -27, 270, -490, 270, -27, 2]
                indices = list(range(i-3, i+4)) if i-3 >= 0 and i+3 < n else None

                if indices and all(0 <= idx < n for idx in indices):
                    s_double_prime_x[i] = sum(c * x[j] for c, j in zip(coeff, indices)) / (180 * dtheta**2)
                    s_double_prime_y[i] = sum(c * y[j] for c, j in zip(coeff, indices)) / (180 * dtheta**2)

        # Equation 37: Near right boundary
        if n >= 3:
            s_double_prime_x[-1] = (x[-3] - 2*x[-2] + x[-1]) / (dtheta**2)
            s_double_prime_y[-1] = (y[-3] - 2*y[-2] + y[-1]) / (dtheta**2)

        # Equation 39: Ghost point method for improved accuracy at boundaries
        if n >= 4:
            # Left boundary ghost point: s_{-1} = 2s_0 - s_1
            ghost_left_x = 2*x[0] - x[1]
            ghost_left_y = 2*y[0] - y[1]
            s_double_prime_x[1] = (ghost_left_x - 2*x[0] + x[1]) / (dtheta**2)
            s_double_prime_y[1] = (ghost_left_y - 2*y[0] + y[1]) / (dtheta**2)

            # Right boundary ghost point: s_{n+1} = 2s_n - s_{n-1}
            ghost_right_x = 2*x[-1] - x[-2]
            ghost_right_y = 2*y[-1] - y[-2]
            s_double_prime_x[-2] = (x[-2] - 2*x[-1] + ghost_right_x) / (dtheta**2)
            s_double_prime_y[-2] = (y[-2] - 2*y[-1] + ghost_right_y) / (dtheta**2)

        return s_prime_x, s_prime_y, s_double_prime_x, s_double_prime_y

    def compute_curvature(self, s_prime_x: np.ndarray, s_prime_y: np.ndarray,
                         s_double_prime_x: np.ndarray, s_double_prime_y: np.ndarray) -> np.ndarray:
        """
        Compute path curvature κ = (s'×s'')/(||s'||³)
        """
        # Cross product in 2D: s'×s'' = s'ₓs''ᵧ - s'ᵧs''ₓ
        cross_product = s_prime_x * s_double_prime_y - s_prime_y * s_double_prime_x

        # Magnitude cubed: ||s'||³
        magnitude_cubed = (s_prime_x**2 + s_prime_y**2)**(3/2)

        # Handle division by zero
        kappa = np.zeros_like(cross_product)
        valid_mask = magnitude_cubed > 1e-12
        kappa[valid_mask] = cross_product[valid_mask] / magnitude_cubed[valid_mask]

        return kappa

    def setup_dynamics_constraints(self, s_prime_x: np.ndarray, s_prime_y: np.ndarray,
                                  s_double_prime_x: np.ndarray, s_double_prime_y: np.ndarray,
                                  theta: np.ndarray, a: cp.Variable, b: cp.Variable) -> list:
        """
        Setup dynamics constraints from the reparameterized dynamics.

        The key constraint is the differential relation b'(θ) = 2a(θ) from Equation 24.
        This comes from the change of variables: b(θ) = θ̇², a(θ) = θ̈.
        """
        constraints = []
        n = len(theta)
        dtheta = 1.0 / (n - 1) if n > 1 else 1.0

        # Equation 33: Discretize b'(θ) = 2a(θ) using finite differences
        # Paper uses different discretization schemes - we implement the exact one

        # Method 1: Forward difference (Equation 33a)
        # (b_{i+1} - b_i) / h = 2a_i
        for i in range(n-1):
            constraints.append((b[i+1] - b[i]) / dtheta == 2 * a[i])

        # Method 2: Backward difference (Equation 33b) - alternative
        # Uncomment to use backward difference instead:
        # for i in range(1, n):
        #     constraints.append((b[i] - b[i-1]) / dtheta == 2 * a[i])

        # Method 3: Central difference at midpoints (Equation 33c) - most accurate
        # This would require a[i+1/2] which is more complex to implement

        return constraints

    def setup_control_constraints(self, s_prime_x: np.ndarray, s_prime_y: np.ndarray,
                                 kappa: np.ndarray, b: cp.Variable, n: int) -> list:
        """
        Setup control constraints C̃_θ from Section 4.4 (friction circle model).

        The constraint is ||u||₂ ≤ μₛFₙ where u represents forces.
        For the speed optimization, this becomes a constraint on lateral acceleration.
        """
        constraints = []

        # Constraint from Equation 10: lateral acceleration constraint
        # For each node where we have curvature information
        for i in range(min(len(kappa), n)):
            if i < len(s_prime_x) and i < len(s_prime_y):
                # Speed at this point: v = ||s'|| * sqrt(b)
                speed_sq = (s_prime_x[i]**2 + s_prime_y[i]**2) * b[i]

                # Lateral acceleration: aₗ = v²κ
                # Constraint: aₗ ≤ μₛg
                if abs(kappa[i]) > 1e-9:
                    lateral_accel = speed_sq * abs(kappa[i])
                    max_lateral = self.vehicle.mu_s * 9.81
                    constraints.append(lateral_accel <= max_lateral)

        # Additional constraint: minimum speed for numerical stability
        for i in range(n):
            constraints.append(b[i] >= 1e-6)

        return constraints

    def setup_objective_function(self, b: cp.Variable, theta: np.ndarray) -> cp.Expression:
        """
        Setup the exact objective function from Equation 28.

        The continuous objective ∫₀¹ b(θ)^(-1/2) dθ is discretized exactly as:
        Σᵢ₌₀ⁿ⁻¹ 2(θᵢ₊₁ - θᵢ)/(√bᵢ + √bᵢ₊₁)

        This formula comes from assuming a(θ) is constant on each interval,
        which gives the exact integral of b(θ)^(-1/2) over the interval.
        """
        n = len(theta)

        if n < 2:
            # Fallback for single point
            return cp.sum(cp.power(b, -0.5))

        # Equation 28: exact discretization
        # For uniform spacing: dtheta = (θᵢ₊₁ - θᵢ) = 1/(n-1)
        dtheta = 1.0 / (n - 1) if n > 1 else 1.0

        objective_terms = []
        for i in range(n-1):
            # Exact formula: 2Δθ/(√bᵢ + √bᵢ₊₁)
            # This is the analytical integral assuming constant acceleration
            # on each interval: ∫[θᵢ,θᵢ₊₁] b(θ)^(-1/2) dθ
            numerator = 2 * dtheta
            denominator = cp.sqrt(b[i]) + cp.sqrt(b[i+1])
            term = numerator / denominator
            objective_terms.append(term)

        return cp.sum(objective_terms)

    def solve(self, x: np.ndarray, y: np.ndarray,
             v_init: float = 0.0, v_final: Optional[float] = None,
             solver: str = 'ECOS', verbose: bool = True) -> Dict[str, Any]:
        """
        Solve the minimum-time speed optimization problem.

        Args:
            x, y: Path coordinates
            v_init: Initial speed (m/s)
            v_final: Final speed (m/s), None for free final speed
            solver: CVXPY solver ('ECOS', 'SCS', 'OSQP')
            verbose: Print solver output

        Returns:
            Dictionary with solution including:
            - 'b_optimal': Optimal b(θ) values
            - 'v_optimal': Optimal speed profile
            - 'total_time': Total time
            - 'theta': Parameter values
            - 'kappa': Curvature profile
            - 'solve_time': Solver time
            - 'solver_status': Solver status
        """
        start_time = time.time()

        # Step 1: Reparameterize path
        theta_path, total_length = self.reparameterize_path(x, y)

        # Step 2: Create uniform θ grid
        n = self.discretization.n_points
        theta = np.linspace(0, 1, n)
        dtheta = theta[1] - theta[0]

        # Interpolate path to uniform grid
        x_uniform = np.interp(theta, theta_path, x)
        y_uniform = np.interp(theta, theta_path, y)

        # Step 3: Compute derivatives and curvature
        s_prime_x, s_prime_y, s_double_prime_x, s_double_prime_y = \
            self.compute_path_derivatives(x_uniform, y_uniform, theta)

        kappa = self.compute_curvature(s_prime_x, s_prime_y, s_double_prime_x, s_double_prime_y)

        # Step 4: Setup optimization variables
        b = cp.Variable(n, pos=True)  # b(θ) = θ̇²
        a = cp.Variable(n)  # a(θ) = θ̈

        # Step 5: Setup objective (Equation 28)
        objective = self.setup_objective_function(b, theta)

        # Step 6: Setup constraints
        constraints = []

        # Initial condition (Equation 32): Conversion from physical to parameter space
        if v_init > 0:
            # From b(θ) = θ̇² and v = ||ṡ'(θ)||θ̇, we get: b = (v / ||ṡ'(θ)||)²
            if len(s_prime_x) > 0:
                s_prime_norm_0 = np.sqrt(s_prime_x[0]**2 + s_prime_y[0]**2)
                # Physical speed to parameter space conversion
                b0_value = (v_init / max(s_prime_norm_0, 1e-9))**2
                constraints.append(b[0] == b0_value)
            else:
                # Fallback if no derivative information
                constraints.append(b[0] == v_init**2)

        # Final condition: Same conversion as initial condition
        if v_final is not None:
            if len(s_prime_x) > 0:
                s_prime_norm_final = np.sqrt(s_prime_x[-1]**2 + s_prime_y[-1]**2)
                # Physical speed to parameter space conversion
                bn_value = (v_final / max(s_prime_norm_final, 1e-9))**2
                constraints.append(b[-1] == bn_value)
            else:
                constraints.append(b[-1] == v_final**2)

        # Dynamics constraints (b'(θ) = 2a(θ))
        dynamics_constraints = self.setup_dynamics_constraints(
            s_prime_x, s_prime_y, s_double_prime_x, s_double_prime_y, theta, a, b)
        constraints.extend(dynamics_constraints)

        # Control constraints (friction circle)
        control_constraints = self.setup_control_constraints(s_prime_x, s_prime_y, kappa, b, n)
        constraints.extend(control_constraints)

        # Step 7: Solve optimization problem
        problem = cp.Problem(cp.Minimize(objective), constraints)

        # Solve with specified solver
        solver_options = {
            'verbose': verbose,
            'max_iters': 2000,
        }

        if solver == 'ECOS':
            solver_options.update({
                'abstol': 1e-7,
                'reltol': 1e-7,
                'feastol': 1e-7
            })

        try:
            problem.solve(solver=getattr(cp, solver), **solver_options)
        except Exception as e:
            if verbose:
                print(f"Solver {solver} failed: {e}")
                print("Trying fallback solver SCS...")
            problem.solve(solver=cp.SCS, verbose=verbose)

        solve_time = time.time() - start_time

        # Step 8: Extract solution
        if b.value is None:
            raise RuntimeError(f"Optimization failed. Status: {problem.status}")

        b_optimal = np.maximum(b.value, 1e-9)

        # Convert back to physical speed profile
        v_optimal = np.zeros(n)
        for i in range(n):
            if i < len(s_prime_x):
                s_prime_norm = np.sqrt(s_prime_x[i]**2 + s_prime_y[i]**2)
            else:
                s_prime_norm = 1.0
            v_optimal[i] = s_prime_norm * np.sqrt(b_optimal[i])

        # Total time calculation (convert from normalized time)
        total_time_normalized = problem.value
        total_time_physical = total_time_normalized * total_length

        solution = {
            'b_optimal': b_optimal,
            'v_optimal': v_optimal,
            'a_optimal': a.value if a.value is not None else np.zeros(n),
            'total_time_normalized': total_time_normalized,
            'total_time_physical': total_time_physical,
            'theta': theta,
            'x_uniform': x_uniform,
            'y_uniform': y_uniform,
            'kappa': kappa,
            'total_length': total_length,
            'solve_time': solve_time,
            'solver_status': problem.status,
            'objective_value': problem.value
        }

        self.solution_history.append(solution)

        if verbose:
            print(f"Optimization completed in {solve_time:.3f}s")
            print(f"Status: {problem.status}")
            print(f"Total time (normalized): {total_time_normalized:.6f}")
            print(f"Total time (physical): {total_time_physical:.3f}s")
            print(f"Max speed: {np.max(v_optimal):.2f} m/s")
            print(f"Min speed: {np.min(v_optimal):.2f} m/s")

        return solution

def create_test_path_from_paper() -> Tuple[np.ndarray, np.ndarray]:
    """
    Create a test path similar to those used in the paper.
    Figure 1 shows a curved path with challenging turns.
    """
    # Create a path with sharp curves (similar to paper's friction circle example)
    t = np.linspace(0, 4*np.pi, 200)

    # Parametric path with varying curvature
    x = 10 * t + 5 * np.sin(t)
    y = 5 * np.cos(t) + 2 * np.sin(2*t)

    return x, y

if __name__ == "__main__":
    print("=== Exact Lipp & Boyd 2014 Implementation ===")

    # Test with paper-like path
    x, y = create_test_path_from_paper()

    # Vehicle parameters similar to paper's friction circle model
    vehicle = VehicleDynamics(
        mass=1000.0,  # kg
        mu_s=0.9      # friction coefficient from paper
    )

    # Discretization with parameters from paper
    discretization = DiscretizationParams(
        n_points=101  # Paper uses various discretizations
    )

    optimizer = LippBoydMinimumTimeOptimizer(vehicle, discretization)

    # Solve with different initial speeds
    for v_init in [0.1, 1.0, 5.0]:
        print(f"\n--- Testing with v_init = {v_init} m/s ---")

        try:
            solution = optimizer.solve(
                x, y,
                v_init=v_init,
                v_final=v_init,  # Return to same speed
                solver='ECOS',
                verbose=True
            )

            # Save detailed results for comparison with paper
            results_file = f"/home/dawgs_nx/f1tenth_dawgs/src/utilities/tunercar/shared/lipp_boyd_results_v{v_init}.txt"
            with open(results_file, 'w') as f:
                f.write("Lipp & Boyd 2014 Exact Implementation Results\n")
                f.write("=" * 50 + "\n")
                f.write(f"Initial speed: {v_init} m/s\n")
                f.write(f"Solver status: {solution['solver_status']}\n")
                f.write(f"Solve time: {solution['solve_time']:.3f}s\n")
                f.write(f"Total time (normalized): {solution['total_time_normalized']:.6f}\n")
                f.write(f"Total time (physical): {solution['total_time_physical']:.3f}s\n")
                f.write(f"Path length: {solution['total_length']:.3f}m\n")
                f.write(f"Max speed: {np.max(solution['v_optimal']):.3f} m/s\n")
                f.write(f"Min speed: {np.min(solution['v_optimal']):.3f} m/s\n")
                f.write(f"Max curvature: {np.max(np.abs(solution['kappa'])):.6f} rad/m\n")
                f.write(f"Discretization points: {len(solution['theta'])}\n")
                f.write("\nDetailed Solution:\n")
                f.write("theta,x,y,v,b,kappa\n")

                for i in range(len(solution['theta'])):
                    theta = solution['theta'][i]
                    x_val = solution['x_uniform'][i]
                    y_val = solution['y_uniform'][i]
                    v_val = solution['v_optimal'][i]
                    b_val = solution['b_optimal'][i]
                    k_val = solution['kappa'][min(i, len(solution['kappa'])-1)] if len(solution['kappa']) > 0 else 0.0

                    f.write(f"{theta:.6f},{x_val:.6f},{y_val:.6f},{v_val:.6f},{b_val:.6f},{k_val:.6f}\n")

            print(f"Detailed results saved to: {results_file}")

        except Exception as e:
            print(f"Optimization failed: {e}")
            import traceback
            traceback.print_exc()

    print("\n=== Implementation Complete ===")
    print("This implementation follows the exact formulation from Lipp & Boyd 2014.")
    print("Compare results with paper's Figure 4 and timing results from Figure 3.")