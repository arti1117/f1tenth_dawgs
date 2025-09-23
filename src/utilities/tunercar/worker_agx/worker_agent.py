import os, pickle, argparse
import zmq
from pathlib import Path
import sys
sys.path.append(str(Path(__file__).resolve().parents[1] / "shared"))
from evaluate_domain import evaluate_theta_in_domain

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--master_ip", required=True, help="NX IP")
    p.add_argument("--job_port", type=int, default=5555)
    p.add_argument("--res_port", type=int, default=5556)
    p.add_argument("--domain_id", type=int, required=True)
    p.add_argument("--params_template", default=str(Path(__file__).resolve().parents[1]/"shared/params_template.yaml"))
    args = p.parse_args()

    ctx = zmq.Context()
    job_receiver = ctx.socket(zmq.PULL); job_receiver.connect(f"tcp://{args.master_ip}:{args.job_port}")
    result_sender = ctx.socket(zmq.PUSH); result_sender.connect(f"tcp://{args.master_ip}:{args.res_port}")

    print(f"[Worker d{args.domain_id}] connected to {args.master_ip}:{args.job_port}/{args.res_port}")

    while True:
        idx, theta = pickle.loads(job_receiver.recv())
        # 간단한 라운드로빈을 마스터가 아니라 worker 도메인 고정으로 처리:
        lap = evaluate_theta_in_domain(theta, args.domain_id, args.params_template)
        result_sender.send(pickle.dumps((idx, lap)))

if __name__ == "__main__":
    main()
