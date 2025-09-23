import numpy as np
import pandas as pd
import os

# --------------------------
# Data Load
# --------------------------

# ---------------------------
# PARAMETERS (edit these to change vehicle/run)
vehicle_mass = 4.3           # kg (editable)
wheel_base = 0.33            # meter
mu = 0.9                     # friction coefficient (editable)
g = 9.81
v_init = 0.0          # m/s initial linear speed at theta0 (editable)
v_max = 15.0
a_max = 6.0                  # initial barrier weight
a_min = -6.0
wheel_positions_body = np.array([  # FL, FR, RL, RR positions [x_forward, y_right]
    [ 0.18,  0.15],
    [ 0.18, -0.15],
    [-0.18,  0.15],
    [-0.18, -0.15]
])
# ---------------------------
# ------- User Inputs -------
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/racetracks/"
map_name = "levine/levine_blacked_"

csv_path = os.path.join(map_dir, map_name + "frenet_track.csv") # centerline
out_path = os.path.join(map_dir, map_name + "forback_speedopted.csv") # centerline

# ---------------------------

def load_path(path):
    if os.path.exists(path):
        df = pd.read_csv(path)
        if {"x","y"}.issubset(df.columns):
            return df["x"].to_numpy(float), df["y"].to_numpy(float)
        else:
            arr = df.to_numpy(float)
            return arr[:,0], arr[:,1]
    else:
        raise FileNotFoundError(f"{path} not found")

def compute_ds(x, y):
    """segmented length ds와 accumulated length s"""
    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.sqrt(dx**2 + dy**2)
    s = np.concatenate([[0.0], np.cumsum(ds)])
    return ds, s

def compute_curvature(x, y):
    """3 point approximation"""
    n = len(x)
    kappa = np.zeros(n)
    for i in range(1, n-1):
        x1,y1 = x[i-1], y[i-1]
        x2,y2 = x[i], y[i]
        x3,y3 = x[i+1], y[i+1]
        num = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)
        den = np.hypot(x2-x1, y2-y1) * np.hypot(x3-x2, y3-y2) * np.hypot(x3-x1, y3-y1)
        if den > 1e-12:
            kappa[i] = 2*num/den
    # 양끝은 이웃 값 복제
    kappa[0] = kappa[1]
    kappa[-1] = kappa[-2]
    return kappa

def forward_backward_speed(x, y,
                           mu=0.9, g=9.81,
                           vmax=15.0,
                           a_acc=4.0, a_brake=-6.0,
                           v_init=0.0):
    """
    Forward/Backward 2-pass velocity profile
    Return: v(speed), runtime T, curvature kappa, distance s
    """
    n = len(x)
    ds, s = compute_ds(x, y)
    kappa = compute_curvature(x, y)

    # 곡률 기반 속도 상한
    v_cap = np.full(n, vmax)
    m = np.abs(kappa) > 1e-9
    v_cap[m] = np.sqrt(mu*g/np.abs(kappa[m]))

    # Forward pass (가속 제한)
    v = np.zeros(n)
    v[0] = v_init
    for i in range(1, n):
        v[i] = min(v_cap[i], np.sqrt(v[i-1]**2 + 2*a_acc*ds[i-1]))

    # Backward pass (제동 제한)
    for i in range(n-2, -1, -1):
        v[i] = min(v[i], np.sqrt(max(v[i+1]**2 - 2*a_brake*ds[i], 0.0)))

    # 근사 목적함수: 총 주행시간
    T = np.sum(ds / np.maximum(v[1:], 1e-6))

    print("ds mean:", np.mean(ds))
    print("kappa min/max:", np.min(kappa), np.max(kappa))
    print("v_cap min/max:", np.min(v_cap), np.max(v_cap))

    return v, T, kappa, s

def main():
    x, y = load_path(csv_path)
    v, T, kappa, s = forward_backward_speed(x, y, vmax=v_max, a_acc=a_max, a_brake=a_min, v_init=v_init)

    df = pd.DataFrame({
        "x": x,
        "y": y,
        "v": v,
        "kappa": kappa,
        "s": s
    })
    df.round({
        'x':3,
        'y':3,
        'kappa':4,
        'v':3,
        "s": 3
    }).to_csv(out_path,index=False)
    print("Saved path with velocity profile: ", out_path)

    print(f"Total time ~ {T:.3f} sec")

if __name__ == "__main__":
    main()
