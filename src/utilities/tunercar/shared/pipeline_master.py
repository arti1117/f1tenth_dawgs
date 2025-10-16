import os, time, signal, atexit
import numpy as np
from concurrent.futures import ThreadPoolExecutor, as_completed
from evaluate_domain import evaluate_theta_in_domain, cleanup_all_domains, monitor_processes
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

def evaluate_population_indexed(X: np.ndarray, generation: int = 0):
    results = [None] * len(X)

    def task(i, x):
        domain_id = DOMAINS[i % len(DOMAINS)]
        print(f"[Gen {generation:02d}] Starting evaluation {i} in domain {domain_id}")
        try:
            r = evaluate_theta_in_domain(x, domain_id, PARAMS_TEMPLATE, None)
            print(f"[Gen {generation:02d}] idx={i}, domain={domain_id}, x={x}, lap_time={r}")
            return r
        except Exception as e:
            print(f"[ERROR] Task {i} in domain {domain_id} failed: {e}")
            return float("inf")

    # Pre-evaluation cleanup
    print(f"[Gen {generation:02d}] Starting generation evaluation...")

    with ThreadPoolExecutor(max_workers=len(DOMAINS)) as ex:
        futures = {ex.submit(task, i, x): i for i, x in enumerate(X)}
        for future in as_completed(futures):
            i = futures[future]
            try:
                results[i] = future.result()
            except Exception as e:
                print(f"[ERROR] Task {i} failed: {e}")
                results[i] = float("inf")

    # Post-evaluation cleanup
    print(f"[Gen {generation:02d}] Generation evaluation completed, cleaning up...")
    time.sleep(2)  # Allow processes to settle
    cleanup_all_domains()
    time.sleep(1)  # Allow cleanup to complete

    fvals = np.array(results, dtype=float)
    fvals = np.nan_to_num(fvals, nan=np.inf, posinf=np.inf, neginf=np.inf)

    print(f"[Gen {generation:02d}] All evaluations completed. Results: {fvals}")
    return fvals

def cleanup_handler(signum, frame):
    """Signal handler for cleanup"""
    print(f"\n[SIGNAL] Received signal {signum}, cleaning up...")
    cleanup_all_domains()
    exit(0)

def main():
    # Register signal handlers for cleanup
    signal.signal(signal.SIGINT, cleanup_handler)
    signal.signal(signal.SIGTERM, cleanup_handler)

    # Register cleanup on exit
    atexit.register(cleanup_all_domains)

    # Initial cleanup
    print("[INIT] Performing initial cleanup...")
    cleanup_all_domains()
    time.sleep(2)

    np.random.seed(0)
    n = 2  # dimension of theta: [lookahead_base, lookahead_k]
    x0 = np.array([1.5, 0.6])
    cfg = CMAESConfig(pop_size=POP_SIZE, max_iter=MAX_ITER, sigma0=0.2)

    es = CMAES(x0, cfg.sigma0, cfg.pop_size, cfg.elite_ratio)
    best_f, best_x = float('inf'), x0.copy()

    print(f"[INIT] CMA-ES started with x0: {x0}")
    print(f"[INIT] Population size: {POP_SIZE}, Max iterations: {MAX_ITER}")
    print(f"[INIT] Domains: {DOMAINS}")

    try:
        for it in range(cfg.max_iter):
            print(f"\n{'='*50}")
            print(f"[Iter {it:02d}] Starting generation {it}")
            print(f"{'='*50}")

            X, z = es.ask()

            # Monitor processes before evaluation
            print(f"[Iter {it:02d}] ROS processes before evaluation:")
            monitor_processes()

            fvals = evaluate_population_indexed(X, generation=it)
            print(f"[Iter {it:02d}] Generation results: {fvals}")

            fmin, xmin = es.tell(X, z, fvals)

            if fmin < best_f:
                best_f, best_x = fmin, xmin
                print(f"[Iter {it:02d}] NEW BEST! {best_f:.3f} with theta {best_x}")

            print(f"[Iter {it:02d}] Best: {best_f:.3f} | Current: {fmin:.3f} | Theta: {xmin}")

            # Monitor processes after evaluation
            print(f"[Iter {it:02d}] ROS processes after evaluation:")
            remaining_procs = monitor_processes()
            if remaining_procs:
                print(f"[WARNING] {len(remaining_procs)} ROS processes still running!")

            # Inter-generation pause
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[INTERRUPT] Optimization interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] Optimization failed: {e}")
    finally:
        print("\n[CLEANUP] Final cleanup...")
        cleanup_all_domains()
        time.sleep(1)

    print("\n" + "="*50)
    print("=== OPTIMIZATION COMPLETED ===")
    print("="*50)
    print(f"Best fitness: {best_f:.6f}")
    print(f"Best theta: {best_x}")
    print(f"Iterations completed: {min(cfg.max_iter, it+1 if 'it' in locals() else 0)}")

    # Final process check
    final_procs = monitor_processes()
    if final_procs:
        print(f"[WARNING] {len(final_procs)} ROS processes still running after cleanup!")
        cleanup_all_domains()
    else:
        print("[SUCCESS] All ROS processes cleaned up successfully")

if __name__ == "__main__":
    main()
