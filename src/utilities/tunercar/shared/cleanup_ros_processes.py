#!/usr/bin/env python3
"""
ROS2 Process Cleanup Utility

This script helps clean up lingering ROS2 processes from CMA-ES optimization runs.
It can be run standalone or imported as a module.

Usage:
    python3 cleanup_ros_processes.py
    python3 cleanup_ros_processes.py --domain 1
    python3 cleanup_ros_processes.py --all
    python3 cleanup_ros_processes.py --monitor
"""

import argparse
import sys
import os
import time
import psutil
import signal

def get_ros_processes():
    """Get all ROS2 processes"""
    processes = []
    try:
        for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'environ', 'create_time']):
            try:
                cmdline = proc.info['cmdline']
                if not cmdline:
                    continue

                cmdline_str = ' '.join(cmdline)

                # Check if it's a ROS2 process
                if 'ros2' in cmdline_str:
                    domain_id = 'N/A'
                    if proc.info['environ']:
                        domain_id = proc.info['environ'].get('ROS_DOMAIN_ID', 'N/A')

                    processes.append({
                        'pid': proc.info['pid'],
                        'name': proc.info['name'],
                        'cmdline': cmdline_str,
                        'domain': domain_id,
                        'create_time': proc.info['create_time'],
                        'proc': proc
                    })

                # Also check for specific node names
                elif any(node in cmdline_str for node in [
                    'pure_pursuit', 'f1tenth_gym', 'lap_listener', 'bridge_launch',
                    'cmaes_bridge', 'gazebo', 'rviz', 'robot_state_publisher'
                ]):
                    domain_id = 'N/A'
                    if proc.info['environ']:
                        domain_id = proc.info['environ'].get('ROS_DOMAIN_ID', 'N/A')

                    processes.append({
                        'pid': proc.info['pid'],
                        'name': proc.info['name'],
                        'cmdline': cmdline_str,
                        'domain': domain_id,
                        'create_time': proc.info['create_time'],
                        'proc': proc
                    })

            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
    except Exception as e:
        print(f"Error getting processes: {e}")

    return processes

def display_processes(processes):
    """Display processes in a formatted table"""
    if not processes:
        print("No ROS2 processes found.")
        return

    print(f"Found {len(processes)} ROS2 processes:")
    print("-" * 100)
    print(f"{'PID':<8} {'Domain':<8} {'Name':<20} {'Age(s)':<8} {'Command':<50}")
    print("-" * 100)

    current_time = time.time()
    for proc in sorted(processes, key=lambda x: x['pid']):
        age = int(current_time - proc['create_time'])
        cmdline = proc['cmdline'][:47] + "..." if len(proc['cmdline']) > 50 else proc['cmdline']
        print(f"{proc['pid']:<8} {proc['domain']:<8} {proc['name']:<20} {age:<8} {cmdline}")

def kill_processes(processes, force=False):
    """Kill the given processes"""
    if not processes:
        print("No processes to kill.")
        return

    killed = 0
    failed = 0

    for proc_info in processes:
        try:
            proc = proc_info['proc']
            pid = proc_info['pid']
            name = proc_info['name']

            if force:
                proc.kill()  # SIGKILL
            else:
                proc.terminate()  # SIGTERM

            print(f"{'Killed' if force else 'Terminated'} {name} (PID: {pid})")
            killed += 1

        except psutil.NoSuchProcess:
            print(f"Process {proc_info['pid']} already terminated")
        except psutil.AccessDenied:
            print(f"Access denied killing process {proc_info['pid']}")
            failed += 1
        except Exception as e:
            print(f"Error killing process {proc_info['pid']}: {e}")
            failed += 1

    print(f"\nSummary: {killed} processes killed, {failed} failed")

    if killed > 0 and not force:
        # Wait a bit and check for remaining processes
        time.sleep(2)
        remaining = get_ros_processes()
        if remaining:
            print(f"\n{len(remaining)} processes still running. Use --force to kill them.")

def kill_by_domain(domain_id):
    """Kill processes in a specific domain"""
    processes = get_ros_processes()
    domain_processes = [p for p in processes if p['domain'] == str(domain_id)]

    if not domain_processes:
        print(f"No processes found in domain {domain_id}")
        return

    print(f"Killing {len(domain_processes)} processes in domain {domain_id}:")
    kill_processes(domain_processes)

def monitor_processes():
    """Monitor processes continuously"""
    try:
        while True:
            os.system('clear' if os.name == 'posix' else 'cls')
            print("ROS2 Process Monitor (Press Ctrl+C to exit)")
            print("=" * 100)

            processes = get_ros_processes()
            display_processes(processes)

            print(f"\nLast updated: {time.strftime('%Y-%m-%d %H:%M:%S')}")
            print("Press Ctrl+C to exit")

            time.sleep(5)

    except KeyboardInterrupt:
        print("\nMonitoring stopped.")

def interactive_cleanup():
    """Interactive cleanup mode"""
    while True:
        processes = get_ros_processes()

        if not processes:
            print("No ROS2 processes found. Exiting.")
            break

        display_processes(processes)

        print("\nOptions:")
        print("1. Kill all processes")
        print("2. Kill by domain")
        print("3. Kill specific processes")
        print("4. Force kill all")
        print("5. Monitor")
        print("6. Refresh")
        print("0. Exit")

        choice = input("\nEnter your choice: ").strip()

        if choice == '0':
            break
        elif choice == '1':
            confirm = input(f"Kill all {len(processes)} processes? (y/N): ").lower()
            if confirm == 'y':
                kill_processes(processes)
        elif choice == '2':
            domain = input("Enter domain ID: ").strip()
            if domain.isdigit():
                kill_by_domain(int(domain))
            else:
                print("Invalid domain ID")
        elif choice == '3':
            pids_input = input("Enter PIDs to kill (space separated): ").strip()
            try:
                pids = [int(p) for p in pids_input.split()]
                selected_procs = [p for p in processes if p['pid'] in pids]
                if selected_procs:
                    kill_processes(selected_procs)
                else:
                    print("No matching processes found")
            except ValueError:
                print("Invalid PID format")
        elif choice == '4':
            confirm = input(f"Force kill all {len(processes)} processes? (y/N): ").lower()
            if confirm == 'y':
                kill_processes(processes, force=True)
        elif choice == '5':
            monitor_processes()
        elif choice == '6':
            continue
        else:
            print("Invalid choice")

        if choice in ['1', '2', '3', '4']:
            input("\nPress Enter to continue...")

def main():
    parser = argparse.ArgumentParser(description='ROS2 Process Cleanup Utility')
    parser.add_argument('--domain', type=int, help='Kill processes in specific domain')
    parser.add_argument('--all', action='store_true', help='Kill all ROS2 processes')
    parser.add_argument('--force', action='store_true', help='Force kill (SIGKILL)')
    parser.add_argument('--monitor', action='store_true', help='Monitor processes continuously')
    parser.add_argument('--list', action='store_true', help='List processes and exit')
    parser.add_argument('--interactive', action='store_true', help='Interactive cleanup mode')

    args = parser.parse_args()

    if args.monitor:
        monitor_processes()
    elif args.domain is not None:
        kill_by_domain(args.domain)
    elif args.all:
        processes = get_ros_processes()
        if processes:
            print(f"Killing all {len(processes)} ROS2 processes...")
            kill_processes(processes, force=args.force)
        else:
            print("No ROS2 processes found.")
    elif args.list:
        processes = get_ros_processes()
        display_processes(processes)
    elif args.interactive:
        interactive_cleanup()
    else:
        # Default: show processes and ask what to do
        processes = get_ros_processes()
        display_processes(processes)

        if processes:
            print("\nOptions:")
            print("  --all         Kill all processes")
            print("  --domain N    Kill processes in domain N")
            print("  --force       Use SIGKILL instead of SIGTERM")
            print("  --monitor     Monitor processes continuously")
            print("  --interactive Interactive cleanup mode")
            print("\nOr run with --help for more options")

if __name__ == '__main__':
    main()