import os, subprocess, time, signal, tempfile, yaml, sys, select
from pathlib import Path
from typing import Dict, Optional

# ------- USER INPUT ------
SIM_LAUNCH = ["ros2", "launch", "f1tenth_gym_ros", "cmaes_bridge_launch.py"]
PP_LAUNCH = ["ros2", "launch", "pure_pursuit", "pure_pursuit_node"]
LISTEN_RUN = ["ros2", "run", "laptime_listener", "lap_listener"]
LAP_TOPIC  = "/lap_time"               # std_msgs/Float64
SIM_STARTUP_SECONDS = 12.0              # launch startup
EVAL_TIMEOUT_SECONDS = 60.0           # lap timeout

# DUMMY = os.getenv("EVAL_DUMMY", "0") == "1"

def write_params_file(base_yaml_path: str, out_path: str, overrides: Dict):
    with open(base_yaml_path, 'r') as f:
        params = yaml.safe_load(f) or {}
    # 깊이 2~3 정도만 덮어쓰는 간단한 병합
    for node_name, wrapper in overrides.items():
        params.setdefault(node_name, {}).setdefault('ros__parameters', {}).update(
            wrapper.get('ros__parameters', {})
        )
    with open(out_path, 'w') as f:
        yaml.safe_dump(params, f)



def run_once_in_domain(domain_id: int, params_file: str) -> float:
    """
    도메인 ID를 설정해 시뮬을 띄우고, lap_listener를 실행해 1회 수신 후 종료.
    """

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)

    # 0) launch f1tenth sim
    cmd_sim = f"source /home/dawgs_nx/f1tenth_dawgs/install/setup.bash \
        && ros2 launch f1tenth_gym_ros cmaes_bridge_launch.py"
    sim = subprocess.Popen(["bash", "-c", cmd_sim],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.STDOUT,
                     text=True,
                     env=env,
                     preexec_fn=os.setsid
                     )

    time.sleep(SIM_STARTUP_SECONDS)

    # 1) launch pp controller

    cmd_pp = f"source /home/dawgs_nx/f1tenth_dawgs/install/setup.bash \
        && ros2 run pure_pursuit pure_pursuit_node \
            --ros-args --params-file /tmp/params_domain{domain_id}.yaml"

    pp = subprocess.Popen(["bash", "-c", cmd_pp],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.STDOUT,
                     text=True,
                     env=env,
                     preexec_fn=os.setsid
                     )

    # 2) run lap_listener
    cmd_lap = f"source /home/dawgs_nx/f1tenth_dawgs/install/setup.bash \
        && ros2 run laptime_listener lap_listener"
    listener = subprocess.Popen(["bash", "-c", cmd_lap],
                     stdout=subprocess.PIPE,
                     stderr=subprocess.STDOUT,
                     text=True,
                     env=env,
                     preexec_fn=os.setsid,

                     bufsize=1   # line-buffered
                     )

    time.sleep(SIM_STARTUP_SECONDS)

    # 첫 줄은 그냥 버림
    _ = listener.stdout.readline()

    lap = float('inf')
    deadline = time.time() + EVAL_TIMEOUT_SECONDS

    while time.time() < deadline:
        rlist, _, _ = select.select([listener.stdout], [], [], 0.1)
        if not rlist:
            continue

        line = listener.stdout.readline()
        if not line:  # EOF
            break

        try:
            lap = float(line.strip())  # 두 번째 줄 읽자마자 lap으로 변환
        except ValueError:
            lap = float('inf')
        break  # 두 번째 줄 읽었으니 바로 종료

    # 4) Terminate processes
    for proc in [listener, pp, sim]:
        if proc is None:
            continue
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            if proc.stdout == subprocess.DEVNULL:
                proc.wait(timeout=3)
            else:
                proc.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            if proc.stdout == subprocess.DEVNULL:
                proc.wait(timeout=3)
            else:
                proc.communicate(timeout=3)
        except Exception:
            pass

    return lap

def evaluate_theta_in_domain(
    theta, domain_id: int, params_template: str, out_dir: Optional[str] = None
) -> float:
    """
    theta: paramter vector e.g) [a, b, c, ...]
    theta = [lookahead_base, lookahead_k, ]
    도메인 domain_id에서 1회 평가하여 lap time 반환.
    """
    # theta -> paramter override
    lookahead_base, lookahead_k = float(theta[0]), float(theta[1])

    # temporary paramter -> /temp/params_domain1.yaml
    if out_dir is None:
        out_dir = tempfile.gettempdir()
    out_yaml = str(Path(out_dir) / f"params_domain{domain_id}.yaml")

    overrides = {
        'pure_pursuit': {
            'ros__parameters': {
                'lookahead_base': lookahead_base,
                'lookahead_k': lookahead_k
            }
        }
    }
    write_params_file(params_template, out_yaml, overrides)

    # print(f"[DEBUG] domain: {domain_id}, lh_b: {lookahead_base:.2f}, lh_k: {lookahead_k:.2f}")

    # evaluate simulation
    lap = run_once_in_domain(domain_id, out_yaml)

    return lap
