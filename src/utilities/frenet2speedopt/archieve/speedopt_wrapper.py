from banded_kkt_speedopt_ackermann import solve_banded_kkt_ackermann

import os
import yaml, time
import pandas as pd
import numpy as np

# ----- User Inputs -----
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/racetracks/"
map_name = "Shanghai/Shanghai_"

yaml_path = os.path.join(map_dir, map_name + "map.yaml")
png_path = os.path.join(map_dir, map_name + "map.png")

csv_path = os.path.join(map_dir, map_name + "frenet_track.csv") # centerline
out_path = os.path.join(map_dir, map_name + "kkt_speedopted.csv") # centerline

# ---------------------------
# PARAMETERS (edit these to change vehicle/run)
vehicle_mass = 3.0           # kg (editable)
wheel_base = 0.33            # meter
mu = 0.9                     # friction coefficient (editable)
g = 9.81
initial_speed = 0.5          # m/s initial linear speed at theta0 (editable)
tau0 = 0.5                   # initial barrier weight
tau_multiplier = 8.0
max_newton_iters = 25
wheel_positions_body = np.array([  # FL, FR, RL, RR positions [x_forward, y_right]
    [ 0.18,  0.15],
    [ 0.18, -0.15],
    [-0.18,  0.15],
    [-0.18, -0.15]
])
# ---------------------------

# 1. Load YAML
with open(yaml_path, 'r') as f:
    map_yaml = yaml.safe_load(f)

resolution = map_yaml["resolution"]
origin = map_yaml["origin"][:2]

# 2. Load path
def load_path(path='path_xy.csv'):
    df = pd.read_csv(path)
    if {'x','y'}.issubset(df.columns):
        x = df['x'].to_numpy()
        y = df['y'].to_numpy()
    else:
        arr = df.to_numpy()
        x, y = arr[:,0], arr[:,1]

    coords = np.column_stack((x, y))
    return coords

# --- calculate curvature ---
def compute_curvature(xs, ys):
    dx = np.gradient(xs)
    dy = np.gradient(ys)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    return curvature

# --- 메인 함수 ---
def optimize_speed_on_raceline(raceline_xy,
                               vehicle_mass=2.0, mu=0.9, g=9.81,
                               wheelbase=0.33, initial_speed=0.0):
    """
    raceline_xy : (n+1,2) numpy array [x,y] raceline points
    vehicle_mass, mu, g : vehicle parameters
    wheelbase : vehicle wheelbase [m]
    initial_speed : starting speed [m/s]

    Returns:
      results : dict with keys {x,y,kappa,v}
    """
    n = raceline_xy.shape[0] - 1
    dtheta = 1.0 / n
    xs, ys = raceline_xy[:,0], raceline_xy[:,1]

    # --- curvature and steerings ---
    curvatures = compute_curvature(xs, ys)
    steering_deltas = np.arctan(wheelbase * curvatures)

    # --- segment lengths ---
    seg_lengths = np.linalg.norm(raceline_xy[1:] - raceline_xy[:-1], axis=1)

    # --- Call Stage A solver ---
    b, a, u = solve_banded_kkt_ackermann(
        raceline_xy, initial_speed, n,
        wheel_positions_body,
        vehicle_mass=vehicle_mass, mu=mu, g=g,
        steering_delta_fun=lambda i: steering_deltas[i],
        tau0 = tau0,
        tau_multiplier = tau_multiplier,
        max_newton_iters = max_newton_iters
    )

    # --- convert to speed ---
    v_theta = np.sqrt(np.maximum(b, 0.0))
    seg_len_nodes = np.zeros_like(b)
    seg_len_nodes[:-1] = seg_lengths
    seg_len_nodes[-1] = seg_lengths[-1]
    v_linear_nodes = v_theta * seg_len_nodes / dtheta

    results = {
        "x": xs,
        "y": ys,
        "kappa": curvatures,
        "v": v_linear_nodes
    }
    return results


def main():
    start_time = time.time()
    raceline = load_path(csv_path)  # shape (n+1, 2)
    v_targets = optimize_speed_on_raceline(raceline, initial_speed=initial_speed, vehicle_mass=vehicle_mass, mu=mu)

    end_time = time.time()

    df_out=pd.DataFrame({
        'x':v_targets['x'],
        'y':v_targets['y'],
        'v':v_targets['v'],
        'kappa':v_targets['kappa']})
    df_out.round({
        'x':3,
        'y':3,
        'kappa':4,
        'v':3
    }).to_csv(out_path,index=False)

    print("Saved to:", out_path)
    print(f"Time spend: {end_time - start_time:.2f}sec")


if __name__=="__main__":
    main()

