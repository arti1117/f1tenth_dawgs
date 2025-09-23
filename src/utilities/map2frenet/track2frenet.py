import os
import numpy as np
import pandas as pd
from scipy.interpolate import splprep, splev

# == User input ==
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/racetracks/"
map_name = "levine/levine_blacked_"

yaml_path = os.path.join(map_dir, map_name + "map.yaml")
png_path = os.path.join(map_dir, map_name + "map.png")
input_csv_path = os.path.join(map_dir, map_name + "track_center.csv")
output_csv_path = os.path.join(map_dir, map_name + "frenet_track.csv")

track_spacing = 0.1

# 1. Load trajectory(csv)
df_in = pd.read_csv(input_csv_path)
traj = df_in[['x', 'y']].to_numpy()
left_dist = df_in['left_dist'].to_numpy()
right_dist = df_in['right_dist'].to_numpy()

# 2. smoothing spline (parameterization based on arc-length)
tck, u = splprep([traj[:,0], traj[:,1]], s=0.5, per=True)  # per=True (closed loop)
unew = np.linspace(0, 1, 2000)  # high resolution
x_smooth, y_smooth = splev(unew, tck)

# 3. Resampling with "track_spacing"
points = np.vstack([x_smooth, y_smooth]).T
dists = np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))
dists = np.insert(dists, 0, 0.0)

track_length = dists[-1]

# 4. Generate target_s (Expect last point -> prevent close loop overlapped)
target_s = np.arange(0, track_length, track_spacing)
if track_length - target_s[-1] < track_spacing * 0.5:
    target_s = target_s[:-1]  # Erase closer than track_spacing * 0.5

# 5. Resampling coordinates
sampled_x = np.interp(target_s, dists, points[:, 0])
sampled_y = np.interp(target_s, dists, points[:, 1])

# 6. Interpolate left/right_dist based on 'arc-length'
# Generate s coordinate with left/right_dist
orig_s = np.linspace(0, track_length, len(left_dist), endpoint=False)
left_dist_resampled = np.interp(target_s, orig_s, left_dist)
right_dist_resampled = np.interp(target_s, orig_s, right_dist)

# 4. derivatives (Frenet quantities)
dx, dy = np.gradient(sampled_x), np.gradient(sampled_y)
ddx, ddy = np.gradient(dx), np.gradient(dy)
dddx, dddy = np.gradient(ddx), np.gradient(ddy)

# arc length step = track_spacing
ds = track_spacing

# curvature κ
kappa = (dx*ddy - dy*ddx) / ( (dx**2 + dy**2)**1.5 + 1e-9 )

# κ′ and κ′′
dkappa = np.gradient(kappa, ds)
ddkappa = np.gradient(dkappa, ds)

# tangent & normal vectors
norm = np.sqrt(dx**2 + dy**2) + 1e-9
Tx, Ty = dx/norm, dy/norm
Nx, Ny = -Ty, Tx

# 5. Save to CSV
df = pd.DataFrame({
    "x": sampled_x,
    "y": sampled_y,
    "left_dist": left_dist_resampled,
    "right_dist": right_dist_resampled,
    "Tx": Tx,
    "Ty": Ty,
    "Nx": Nx,
    "Ny": Ny,
    "kappa": kappa,
    "dkappa": dkappa,
    "ddkappa": ddkappa,
    "s": target_s
})
df.round({
    "x": 4,
    "y": 4,
    "left_dist": 3,
    "right_dist": 3,
    "Tx": 6,
    "Ty": 6,
    "Nx": 6,
    "Ny": 6,
    "kappa": 8,
    "dkappa": 8,
    "ddkappa": 8,
    "s": 3
}).to_csv(output_csv_path, index=False)

print("Saved frenet_frame.csv with", len(df), "points.")
