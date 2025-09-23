# ├─ master_nx/
# │  ├─ master_zmq.py               # NX에서 실행: CMA-ES + ZeroMQ 분산
# │  └─ config.py                   # 마스터 설정 (IP/포트/도메인)

import os, pickle, time, threading
import numpy as np
import zmq
from pathlib import Path
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "shared"))
sys.path.append(str(Path(__file__).resolve().parents[1] / "shared"))
from cma_es_optimizer import CMAES, CMAESConfig
from evaluate_domain import evaluate_theta_in_domain

from config import MASTER_BIND_IP, JOB_PORT, RESULT_PORT, PARAMS_TEMPLATE, LOCAL_DOMAINS

"""
NX가 Master:
- ZeroMQ PUSH(bind tcp://*:5555) 로 잡을 뿌림
- ZeroMQ PULL(bind tcp://*:5556) 로 결과를 수집
- (옵션) NX 내부에서도 LOCAL_DOMAINS로 로컬 워커를 스레드로 돌림
"""

def start_local_worker_thread(domain_id: int, job_sub, result_pub):
    # 로컬 워커: SUB 소켓 대신 간단히 파이프-큐가 없으니, 마스터가 job을 브로드캐스트하지 않고
    # 여기서는 "잡 큐 공유"가 없으므로, 로컬 워커는 마스터가 직접 호출하는 방식으로 단순화.
    # → 더 간단하게: NX 로컬 워커도 ZeroMQ 기반으로 master에 connect하도록 run_nx_workers.sh를 쓰세요.
    pass  # 권장 방식: worker_agent.py를 NX에서도 실행

def distribute_and_collect(es, cfg, context):
    job_sender = context.socket(zmq.PUSH); job_sender.bind(f"tcp://{MASTER_BIND_IP}:{JOB_PORT}")
    result_receiver = context.socket(zmq.PULL); result_receiver.bind(f"tcp://{MASTER_BIND_IP}:{RESULT_PORT}")

    n = es.n
    best_f, best_x = float('inf'), es.mean.copy()

    for it in range(cfg.max_iter):
        X, z = es.ask()
        # 후보 전송
        for i, theta in enumerate(X):
            job_sender.send(pickle.dumps((i, theta.tolist())))
        # 결과 수집
        results = [None] * len(X)
        got = 0
        while got < len(X):
            idx, lap = pickle.loads(result_receiver.recv())
            results[idx] = lap; got += 1

        fvals = np.array(results, dtype=float)
        fmin, xmin = es.tell(X, z, fvals)
        if fmin < best_f:
            best_f, best_x = fmin, xmin
        print(f"[Iter {it:02d}] best {best_f:.3f}  curr {fmin:.3f}  theta {xmin}")

    print("=== Done ===")
    print("Best:", best_f, best_x)

def main():
    # 빠른 연결 테스트: export EVAL_DUMMY=1
    np.random.seed(0)
    n = 2
    x0 = np.array([1.2, 1.0])
    cfg = CMAESConfig(pop_size=12, max_iter=15, sigma0=0.2)
    es = CMAES(x0, cfg.sigma0, cfg.pop_size, cfg.elite_ratio)

    context = zmq.Context()
    distribute_and_collect(es, cfg, context)

if __name__ == "__main__":
    main()
