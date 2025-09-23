# cma_es_optimizer.py
import numpy as np
from dataclasses import dataclass
from typing import Callable, Tuple

@dataclass
class CMAESConfig:
    pop_size: int = 64         # λ
    elite_ratio: float = 0.2   # μ/λ
    sigma0: float = 0.5
    tol_cov_norm: float = 1e-2
    max_iter: int = 200

class CMAES:
    def __init__(self, x0: np.ndarray, sigma0: float, pop_size: int, elite_ratio: float):
        self.n = x0.size
        self.mean = x0.copy()
        self.sigma = sigma0
        self.lambda_ = pop_size
        self.mu = max(1, int(elite_ratio * pop_size))
        self.weights = np.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.weights /= np.sum(self.weights)
        self.mu_eff = 1.0 / np.sum(self.weights**2)

        self.C = np.eye(self.n)
        self.pc = np.zeros(self.n)
        self.ps = np.zeros(self.n)
        self.cc = (4 + self.mu_eff / self.n) / (self.n + 4 + 2*self.mu_eff/self.n)
        self.cs = (self.mu_eff + 2) / (self.n + self.mu_eff + 5)
        self.c1 = 2 / ((self.n + 1.3)**2 + self.mu_eff)
        self.cmu = min(1 - self.c1, 2 * (self.mu_eff - 2 + 1/self.mu_eff) / ((self.n + 2)**2 + self.mu_eff))
        self.damps = 1 + 2*max(0, np.sqrt((self.mu_eff-1)/(self.n+1)) - 1) + self.cs
        self.B = np.eye(self.n)
        self.D = np.ones(self.n)
        self.inv_sqrt_C = np.eye(self.n)
        self.eig_updated = True

    def _update_eigensystem(self):
        D2, B = np.linalg.eigh(self.C)
        self.D = np.sqrt(np.maximum(D2, 1e-20))
        self.B = B
        self.inv_sqrt_C = self.B @ np.diag(1.0 / self.D) @ self.B.T
        self.eig_updated = True

    def ask(self, rng=np.random):
        if not self.eig_updated:
            self._update_eigensystem()
        z = rng.randn(self.lambda_, self.n)
        y = z @ (self.B * self.D).T  # (λ x n)
        X = self.mean + self.sigma * y
        return X, z

    def tell(self, X: np.ndarray, z: np.ndarray, fvals: np.ndarray):
        idx = np.argsort(fvals)
        X = X[idx]; z = z[idx]; fvals = fvals[idx]
        X_elite = X[:self.mu]
        z_elite = z[:self.mu]

        y_w = (X_elite - self.mean) / self.sigma
        y_w = (y_w.T @ self.weights).T
        z_w = (z_elite.T @ self.weights).T

        # mean & paths
        self.mean = self.mean + self.sigma * y_w
        self.ps = (1 - self.cs) * self.ps + np.sqrt(self.cs * (2 - self.cs) * self.mu_eff) * (self.inv_sqrt_C @ z_w)
        hsig = float(np.linalg.norm(self.ps) / np.sqrt(1 - (1 - self.cs)**(2)) / np.sqrt(self.n) < (1.4 + 2/(self.n+1)))
        self.pc = (1 - self.cc) * self.pc + hsig * np.sqrt(self.cc * (2 - self.cc) * self.mu_eff) * y_w

        # covariance
        artmp = (X_elite - self.mean) / self.sigma
        C_mu = sum(self.weights[i] * np.outer(artmp[i], artmp[i]) for i in range(self.mu))
        self.C = ((1 - self.c1 - self.cmu) * self.C
                  + self.c1 * (np.outer(self.pc, self.pc) + (1 - hsig) * self.cc * (2 - self.cc) * self.C)
                  + self.cmu * C_mu)

        # step-size
        self.sigma *= np.exp((self.cs / self.damps) * (np.linalg.norm(self.ps) / np.sqrt(self.n) - 1))

        # mark eig outdated occasionally for speed
        self.eig_updated = False

        return fvals[0], X[0]

def run_cmaes(
    x0: np.ndarray,
    bounds: Tuple[np.ndarray, np.ndarray],
    eval_fn: Callable[[np.ndarray], float],
    cfg: CMAESConfig = CMAESConfig(),
    rng=np.random
):
    es = CMAES(x0, cfg.sigma0, cfg.pop_size, cfg.elite_ratio)
    best_f, best_x = float('inf'), x0.copy()
    for it in range(cfg.max_iter):
        X, z = es.ask(rng)
        # clamp to bounds
        X = np.minimum(np.maximum(X, bounds[0]), bounds[1])
        fvals = np.array([eval_fn(xi) for xi in X])
        fmin, xmin = es.tell(X, z, fvals)
        if fmin < best_f:
            best_f, best_x = fmin, xmin

        # stop if covariance "shrunk"
        cov_norm = np.linalg.norm(es.C)
        if cov_norm < cfg.tol_cov_norm:
            break
    return best_x, best_f
