import os, time
import numpy as np
from concurrent.futures import ThreadPoolExecutor, as_completed
from evaluate_domain import evaluate_theta_in_domain
from cma_es_optimizer import CMAES, CMAESConfig

# 여러분 환경에 맞게
PARAMS_TEMPLATE = "/home/dawgs_nx/f1tenth_dawgs/src/utilities/tunercar/shared/params_template.yaml"
DOMAINS = [1, 2, 3, 4]     # 동시에 돌릴 ROS_DOMAIN_ID
POP_SIZE = 16              # CMAES 인구수 (DOMAINS의 배수면 딱 맞음)
MAX_ITER = 50

def evaluate_population(X: np.ndarray):
    """
    X shape: (λ, n). 후보들을 DOMAINS에 라운드로빈으로 배정해서 병렬 평가.
    """
    futures = []
    with ThreadPoolExecutor(max_workers=len(DOMAINS)) as ex:
        for i, x in enumerate(X):
            domain_id = DOMAINS[i % len(DOMAINS)]
            futures.append(ex.submit(
                evaluate_theta_in_domain,
                x, domain_id, PARAMS_TEMPLATE, None
            ))
        results = [f.result() for f in as_completed(futures)]
    # futures는 완료 순서대로 나오므로, 원래 인덱스로 재정렬
    # 위에선 순서 매핑을 따로 저장하지 않았으니 간단히 다시 평가해서 정렬하는 대신,
    # 더 깔끔하게 인덱스를 묶어 넘기자.
    # → 개선: 인덱스와 함께 넘기기
    return results

def evaluate_population_indexed(X: np.ndarray):
    results = [None] * len(X)

    def task(i, x):
        domain_id = DOMAINS[i % len(DOMAINS)]
        r = evaluate_theta_in_domain(x, domain_id, PARAMS_TEMPLATE, None)
        print(f"[DEBUG] idx={i}, domain={domain_id}, x={x}, lap_time={r}")
        return r

    with ThreadPoolExecutor(max_workers=len(DOMAINS)) as ex:
        futures = {ex.submit(task, i, x): i for i, x in enumerate(X)}
        for future in as_completed(futures):
            i = futures[future]
            try:
                results[i] = future.result()
            except Exception as e:
                print(f"[ERROR] Task {i} failed: {e}")
                results[i] = float("inf")

    fvals = np.array(results, dtype=float)
    fvals = np.nan_to_num(fvals, nan=np.inf, posinf=np.inf, neginf=np.inf)

    return fvals

def main():
    np.random.seed(0)
    n = 2  # dimension of theta: [lookahead_base, lookahead_k]
    x0 = np.array([1.5, 0.6])
    cfg = CMAESConfig(pop_size=POP_SIZE, max_iter=MAX_ITER, sigma0=0.2)

    es = CMAES(x0, cfg.sigma0, cfg.pop_size, cfg.elite_ratio)
    best_f, best_x = float('inf'), x0.copy()

    print(f"cmaes_started x: {x0}")
    for it in range(cfg.max_iter):
        X, z = es.ask()
        fvals = evaluate_population_indexed(X)  # parallel evaluation
        print(f"[DEBUG] fvals: {fvals}")
        fmin, xmin = es.tell(X, z, fvals)

        if fmin < best_f:
            best_f, best_x = fmin, xmin

        print(f"[DEBUG] [Iter {it:02d}] best {best_f:.3f}  | current {fmin:.3f}  | theta {xmin}")
    print("=== Done ===")
    print("Best:", best_f, best_x)

if __name__ == "__main__":
    main()
