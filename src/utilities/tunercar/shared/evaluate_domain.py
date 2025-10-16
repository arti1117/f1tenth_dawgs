import os, subprocess, time, signal, tempfile, yaml, sys, select, psutil
from pathlib import Path
from typing import Dict, Optional, List
import threading
import atexit

# ------- USER INPUT ------
SIM_LAUNCH = ["ros2", "launch", "f1tenth_gym_ros", "cmaes_bridge_launch.py"]
PP_LAUNCH = ["ros2", "launch", "pure_pursuit", "pure_pursuit_node"]
LISTEN_RUN = ["ros2", "run", "laptime_listener", "lap_listener"]
LAP_TOPIC  = "/lap_time"               # std_msgs/Float64
SIM_STARTUP_SECONDS = 12.0              # launch startup
EVAL_TIMEOUT_SECONDS = 60.0           # lap timeout

# DUMMY = os.getenv("EVAL_DUMMY", "0") == "1"

# Global process registry for cleanup
_active_processes = []
_cleanup_lock = threading.Lock()

def register_process(proc):
    """Register a process for cleanup"""
    with _cleanup_lock:
        _active_processes.append(proc)

def unregister_process(proc):
    """Unregister a process from cleanup"""
    with _cleanup_lock:
        if proc in _active_processes:
            _active_processes.remove(proc)

def cleanup_all_processes():
    """Clean up all registered processes"""
    with _cleanup_lock:
        processes_to_clean = _active_processes.copy()
        _active_processes.clear()

    for proc in processes_to_clean:
        try:
            terminate_process_tree(proc)
        except Exception:
            pass

# Register cleanup on exit
atexit.register(cleanup_all_processes)

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

def kill_processes_by_domain(domain_id: int):
    """Kill all ROS2 processes in a specific domain"""
    try:
        # Find all processes with ROS_DOMAIN_ID
        for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'environ']):
            try:
                env = proc.info['environ']
                if env and env.get('ROS_DOMAIN_ID') == str(domain_id):
                    print(f"[DEBUG] Killing domain {domain_id} process: {proc.info['name']} (PID: {proc.info['pid']})")
                    proc.terminate()

                # Also kill processes with ros2 command that might be related
                cmdline = proc.info['cmdline']
                if cmdline and 'ros2' in ' '.join(cmdline):
                    # Check if it's our process by looking for specific nodes
                    cmdline_str = ' '.join(cmdline)
                    if any(node in cmdline_str for node in ['pure_pursuit', 'f1tenth_gym', 'lap_listener']):
                        print(f"[DEBUG] Killing ROS2 process: {proc.info['name']} (PID: {proc.info['pid']})")
                        proc.terminate()

            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue

        # Wait a bit and force kill if necessary
        time.sleep(1)
        for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'environ']):
            try:
                env = proc.info['environ']
                if env and env.get('ROS_DOMAIN_ID') == str(domain_id):
                    proc.kill()

            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue

    except Exception as e:
        print(f"[ERROR] Failed to kill domain {domain_id} processes: {e}")

def terminate_process_tree(proc, timeout=5):
    """Terminate a process and all its children"""
    if proc is None:
        return

    try:
        # Get process tree
        parent = psutil.Process(proc.pid)
        children = parent.children(recursive=True)

        # Terminate children first
        for child in children:
            try:
                child.terminate()
            except psutil.NoSuchProcess:
                pass

        # Terminate parent
        try:
            parent.terminate()
        except psutil.NoSuchProcess:
            return

        # Wait for termination
        gone, still_alive = psutil.wait_procs(children + [parent], timeout=timeout)

        # Force kill remaining processes
        for proc_alive in still_alive:
            try:
                proc_alive.kill()
            except psutil.NoSuchProcess:
                pass

    except psutil.NoSuchProcess:
        pass
    except Exception as e:
        print(f"[ERROR] Failed to terminate process tree: {e}")
        try:
            proc.kill()
        except:
            pass



def run_once_in_domain(domain_id: int, params_file: str) -> float:
    """
    도메인 ID를 설정해 시뮬을 띄우고, lap_listener를 실행해 1회 수신 후 종료.
    개선된 프로세스 관리로 좀비 프로세스 방지.
    """

    processes = []
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)

    print(f"[DEBUG] Starting evaluation in domain {domain_id}")

    try:
        # 기존 도메인의 프로세스들 정리
        kill_processes_by_domain(domain_id)
        time.sleep(1)

        # 0) launch f1tenth sim
        cmd_sim = f"source /home/dawgs_nx/f1tenth_dawgs/install/setup.bash && ros2 launch f1tenth_gym_ros cmaes_bridge_launch.py"
        sim = subprocess.Popen(
            ["bash", "-c", cmd_sim],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            env=env,
            preexec_fn=os.setsid,
            start_new_session=True  # 새로운 세션 시작
        )
        processes.append(sim)
        register_process(sim)
        print(f"[DEBUG] Started sim process PID: {sim.pid}")

        time.sleep(SIM_STARTUP_SECONDS)

        # 1) launch pp controller
        cmd_pp = f"source /home/dawgs_nx/f1tenth_dawgs/install/setup.bash && ros2 run pure_pursuit pure_pursuit_node --ros-args --params-file {params_file}"
        pp = subprocess.Popen(
            ["bash", "-c", cmd_pp],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            env=env,
            preexec_fn=os.setsid,
            start_new_session=True
        )
        processes.append(pp)
        register_process(pp)
        print(f"[DEBUG] Started PP process PID: {pp.pid}")

        # 2) run lap_listener
        cmd_lap = f"source /home/dawgs_nx/f1tenth_dawgs/install/setup.bash && ros2 run laptime_listener lap_listener"
        listener = subprocess.Popen(
            ["bash", "-c", cmd_lap],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=env,
            preexec_fn=os.setsid,
            bufsize=1,  # line-buffered
            start_new_session=True
        )
        processes.append(listener)
        register_process(listener)
        print(f"[DEBUG] Started listener process PID: {listener.pid}")

        time.sleep(2)  # 추가 안정화 시간

        # 첫 줄은 버림
        try:
            first_line = listener.stdout.readline()
            print(f"[DEBUG] First line from listener: {first_line.strip()}")
        except:
            pass

        lap = float('inf')
        deadline = time.time() + EVAL_TIMEOUT_SECONDS

        while time.time() < deadline:
            # Check if processes are still running
            if not all(proc.poll() is None for proc in [sim, pp, listener]):
                print(f"[DEBUG] One or more processes died unexpectedly")
                break

            rlist, _, _ = select.select([listener.stdout], [], [], 0.1)
            if not rlist:
                continue

            line = listener.stdout.readline()
            if not line:  # EOF
                print(f"[DEBUG] EOF from listener")
                break

            try:
                lap_value = float(line.strip())
                lap = lap_value
                print(f"[DEBUG] Got lap time: {lap}")
            except ValueError:
                print(f"[DEBUG] Failed to parse lap time: {line.strip()}")
                lap = float('inf')
            break  # 두 번째 줄 읽었으니 바로 종료

    except Exception as e:
        print(f"[ERROR] Exception in run_once_in_domain: {e}")
        lap = float('inf')

    finally:
        # Cleanup processes
        print(f"[DEBUG] Cleaning up processes for domain {domain_id}")

        # First, try graceful termination
        for proc in processes:
            try:
                unregister_process(proc)
                terminate_process_tree(proc, timeout=3)
            except Exception as e:
                print(f"[DEBUG] Error terminating process: {e}")

        # Wait a bit and then force kill by domain
        time.sleep(1)
        kill_processes_by_domain(domain_id)

        # Final cleanup - kill any remaining ROS2 processes
        try:
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = proc.info['cmdline']
                    if cmdline and 'ros2' in ' '.join(cmdline):
                        cmdline_str = ' '.join(cmdline)
                        if any(node in cmdline_str for node in ['pure_pursuit', 'f1tenth_gym', 'lap_listener']):
                            proc.kill()
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
        except Exception:
            pass

        print(f"[DEBUG] Cleanup completed for domain {domain_id}")

    return lap

def cleanup_all_domains():
    """Clean up all processes in all domains"""
    print("[DEBUG] Cleaning up all domains...")
    for domain_id in range(0, 100):  # Clean up domains 0-99
        kill_processes_by_domain(domain_id)

def get_running_ros_processes():
    """Get list of running ROS processes"""
    ros_processes = []
    try:
        for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'environ']):
            try:
                cmdline = proc.info['cmdline']
                if cmdline and 'ros2' in ' '.join(cmdline):
                    ros_processes.append({
                        'pid': proc.info['pid'],
                        'name': proc.info['name'],
                        'cmdline': ' '.join(cmdline),
                        'domain': proc.info['environ'].get('ROS_DOMAIN_ID', 'N/A') if proc.info['environ'] else 'N/A'
                    })
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
    except Exception as e:
        print(f"[ERROR] Failed to get ROS processes: {e}")

    return ros_processes

def monitor_processes():
    """Monitor and display running ROS processes"""
    processes = get_running_ros_processes()
    if processes:
        print(f"[DEBUG] Found {len(processes)} ROS processes:")
        for proc in processes:
            print(f"  PID: {proc['pid']}, Domain: {proc['domain']}, Name: {proc['name']}")
    else:
        print("[DEBUG] No ROS processes found")

    return processes

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
