"""
Convex optimization pipeline for tunercar parameter tuning.
Integrates minimum-time speed optimization with parameter tuning.
Based on Lipp & Boyd 2014 and TUNERCAR framework.
"""

import os
import time
import numpy as np
from concurrent.futures import ThreadPoolExecutor, as_completed
from evaluate_domain import evaluate_theta_in_domain
from convex_solver import (
    ConvexSolver, ConvexConfig, VehicleParams, MinimumTimeSpeedOptimizer,
    solve_tunercar_convex, optimize_racing_line_and_speed
)

# Configuration
PARAMS_TEMPLATE = "/home/dawgs_nx/f1tenth_dawgs/src/utilities/tunercar/shared/params_template.yaml"
DOMAINS = [1, 2, 3, 4]     # ROS_DOMAIN_ID for parallel evaluation
MAX_ITER = 20              # Maximum iterations for trust region method

class ConvexTunerPipeline:
    """
    Convex optimization pipeline for tunercar parameter tuning.
    Uses trust region methods to handle non-convex lap time function.
    """

    def __init__(self, domains=DOMAINS, params_template=PARAMS_TEMPLATE):
        self.domains = domains
        self.params_template = params_template
        self.best_f = float('inf')
        self.best_x = None
        self.history = []

    def evaluate_single(self, x: np.ndarray, domain_id: int) -> float:
        """Evaluate single parameter configuration"""
        try:
            result = evaluate_theta_in_domain(
                x, domain_id, self.params_template, None
            )
            print(f"[DEBUG] domain={domain_id}, x={x}, lap_time={result}")
            return result
        except Exception as e:
            print(f"[ERROR] Evaluation failed for domain {domain_id}: {e}")
            return float('inf')

    def evaluate_parallel(self, X: np.ndarray) -> np.ndarray:
        """
        Evaluate multiple parameter configurations in parallel
        X shape: (n_candidates, n_params)
        """
        results = [None] * len(X)

        def task(i, x):
            domain_id = self.domains[i % len(self.domains)]
            return self.evaluate_single(x, domain_id)

        with ThreadPoolExecutor(max_workers=len(self.domains)) as executor:
            futures = {executor.submit(task, i, x): i for i, x in enumerate(X)}

            for future in as_completed(futures):
                i = futures[future]
                try:
                    results[i] = future.result()
                except Exception as e:
                    print(f"[ERROR] Task {i} failed: {e}")
                    results[i] = float('inf')

        fvals = np.array(results, dtype=float)
        fvals = np.nan_to_num(fvals, nan=np.inf, posinf=np.inf, neginf=np.inf)

        return fvals

    def objective_with_smoothing(self, x: np.ndarray, n_samples: int = 3) -> float:
        """
        Evaluate objective with multiple samples for noise reduction.
        Uses same domain for all samples to ensure consistency.
        """
        domain_id = self.domains[0]  # Use first domain for consistency

        samples = []
        for _ in range(n_samples):
            result = self.evaluate_single(x, domain_id)
            if np.isfinite(result):
                samples.append(result)

        if not samples:
            return float('inf')

        # Return median to reduce noise
        return np.median(samples)

    def run_convex_optimization(
        self,
        x0: np.ndarray,
        bounds: tuple,
        config: ConvexConfig = ConvexConfig(),
        max_iter: int = MAX_ITER,
        trust_radius_init: float = 0.3,
        trust_radius_min: float = 0.05,
        trust_radius_max: float = 1.0,
        shrink_factor: float = 0.5,
        expand_factor: float = 2.0
    ) -> tuple:
        """
        Run convex optimization with adaptive trust region.

        Args:
            x0: Initial parameters [lookahead_base, lookahead_k]
            bounds: Parameter bounds (lower, upper)
            config: Convex solver configuration
            max_iter: Maximum iterations
            trust_radius_init: Initial trust region radius
            trust_radius_min: Minimum trust region radius
            trust_radius_max: Maximum trust region radius
            shrink_factor: Factor to shrink trust region on poor steps
            expand_factor: Factor to expand trust region on good steps
        """

        solver = ConvexSolver(config)

        x_current = x0.copy()
        f_current = self.objective_with_smoothing(x_current)
        trust_radius = trust_radius_init

        self.best_x = x_current.copy()
        self.best_f = f_current

        print(f"[INFO] Starting convex optimization")
        print(f"[INFO] Initial: x={x_current}, f={f_current:.3f}")

        for iteration in range(max_iter):
            print(f"\n[INFO] Iteration {iteration+1}/{max_iter}")
            print(f"[INFO] Current: x={x_current}, f={f_current:.3f}, trust_radius={trust_radius:.3f}")

            try:
                # Solve trust region subproblem
                x_trial, f_approx = solver.local_convex_approximation(
                    x_current, bounds, self.objective_with_smoothing, trust_radius
                )

                # Evaluate actual objective at trial point
                f_trial = self.objective_with_smoothing(x_trial)

                # Compute actual vs predicted reduction
                actual_reduction = f_current - f_trial
                predicted_reduction = f_current - f_approx

                # Compute ratio for trust region update
                if abs(predicted_reduction) < 1e-12:
                    ratio = 0
                else:
                    ratio = actual_reduction / predicted_reduction

                print(f"[DEBUG] Trial: x={x_trial}, f_trial={f_trial:.3f}")
                print(f"[DEBUG] Reductions: actual={actual_reduction:.3f}, predicted={predicted_reduction:.3f}, ratio={ratio:.3f}")

                # Update trust region and accept/reject step
                if ratio > 0.75 and abs(np.linalg.norm(x_trial - x_current) - trust_radius) < 1e-6:
                    # Expand trust region for very successful step
                    trust_radius = min(trust_radius * expand_factor, trust_radius_max)
                    print(f"[DEBUG] Expanding trust region to {trust_radius:.3f}")

                if ratio > 0.1:  # Accept step
                    x_current = x_trial
                    f_current = f_trial

                    if f_current < self.best_f:
                        self.best_f = f_current
                        self.best_x = x_current.copy()
                        print(f"[INFO] New best: f={self.best_f:.3f}, x={self.best_x}")

                    if ratio < 0.25:
                        # Shrink trust region for marginal step
                        trust_radius = max(trust_radius * shrink_factor, trust_radius_min)
                        print(f"[DEBUG] Shrinking trust region to {trust_radius:.3f}")

                else:  # Reject step
                    trust_radius = max(trust_radius * shrink_factor, trust_radius_min)
                    print(f"[DEBUG] Rejecting step, shrinking trust region to {trust_radius:.3f}")

                # Store history
                self.history.append({
                    'iteration': iteration,
                    'x': x_current.copy(),
                    'f': f_current,
                    'trust_radius': trust_radius,
                    'ratio': ratio
                })

                # Convergence check
                if trust_radius < trust_radius_min * 1.1:
                    print(f"[INFO] Converged: trust region too small ({trust_radius:.6f})")
                    break

                if iteration > 0 and abs(self.history[-1]['f'] - self.history[-2]['f']) < 1e-4:
                    print(f"[INFO] Converged: objective change too small")
                    break

            except Exception as e:
                print(f"[ERROR] Iteration {iteration} failed: {e}")
                trust_radius = max(trust_radius * shrink_factor, trust_radius_min)
                if trust_radius < trust_radius_min * 1.1:
                    break

        print(f"\n[INFO] Optimization complete")
        print(f"[INFO] Best result: f={self.best_f:.3f}, x={self.best_x}")

        return self.best_x, self.best_f

def main():
    """Main optimization pipeline"""
    np.random.seed(42)

    # Parameter configuration
    n = 2  # [lookahead_base, lookahead_k]
    x0 = np.array([1.5, 0.6])  # Initial guess
    bounds = (
        np.array([0.5, 0.1]),  # Lower bounds
        np.array([3.0, 1.0])   # Upper bounds
    )

    # Convex solver configuration
    config = ConvexConfig(
        max_iter=1000,
        solver='ECOS',
        abstol=1e-6,
        reltol=1e-6,
        verbose=False
    )

    # Create pipeline
    pipeline = ConvexTunerPipeline()

    print("=== Convex Optimization Pipeline ===")
    print(f"Initial parameters: {x0}")
    print(f"Parameter bounds: {bounds}")
    print(f"Domains: {pipeline.domains}")

    # Run optimization
    start_time = time.time()

    try:
        best_x, best_f = pipeline.run_convex_optimization(
            x0, bounds, config,
            max_iter=MAX_ITER,
            trust_radius_init=0.3
        )

        elapsed = time.time() - start_time

        print("\n=== Results ===")
        print(f"Optimization time: {elapsed:.2f} seconds")
        print(f"Best parameters: lookahead_base={best_x[0]:.3f}, lookahead_k={best_x[1]:.3f}")
        print(f"Best lap time: {best_f:.3f} seconds")
        print(f"Total evaluations: {len(pipeline.history)}")

        # Save results
        result_file = "convex_optimization_results.txt"
        with open(result_file, 'w') as f:
            f.write(f"Convex Optimization Results\n")
            f.write(f"==========================\n")
            f.write(f"Initial parameters: {x0}\n")
            f.write(f"Best parameters: {best_x}\n")
            f.write(f"Best lap time: {best_f:.6f}\n")
            f.write(f"Optimization time: {elapsed:.2f} seconds\n")
            f.write(f"Total iterations: {len(pipeline.history)}\n")
            f.write(f"\nHistory:\n")
            for entry in pipeline.history:
                f.write(f"Iter {entry['iteration']:2d}: f={entry['f']:8.3f}, x={entry['x']}, tr={entry['trust_radius']:.3f}\n")

        print(f"Results saved to: {result_file}")

    except Exception as e:
        print(f"[ERROR] Optimization failed: {e}")
        return None, None

    return best_x, best_f

if __name__ == "__main__":
    main()