import os
import cv2, yaml
import pandas as pd
import numpy as np

# == User input ==
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/racetracks/"
map_name = "Shanghai/Shanghai_"

yaml_path = os.path.join(map_dir, map_name + "map.yaml")
png_path = os.path.join(map_dir, map_name + "map.png")

csv_path = os.path.join(map_dir, map_name + "track_center.csv") # reference track center

out_png_path = os.path.join(map_dir, map_name + "frenet_eval.png") # output image

# 1. Load YAML
with open(yaml_path, 'r') as f:
    map_yaml = yaml.safe_load(f)

resolution = map_yaml["resolution"]
origin = map_yaml["origin"][:2]

# 2. Load map
img = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
free_space = (binary == 255).astype(np.uint8)

# 3. Load csv file
df = pd.read_csv(csv_path)
centerline = df.iloc[:, :2].to_numpy()

def world_to_pixel(x, y, origin, resolution, img_height):
    u = int((x - origin[0]) / resolution)
    v = int(img_height - (y - origin[1]) / resolution)
    return u, v

def compute_normals(centerline):
    """Computes unit normal vectors from given centerline points"""
    tangents = np.diff(centerline, axis=0)
    tangents = np.vstack([tangents, tangents[-1]])  # last calibration (?)
    tangents /= np.linalg.norm(tangents, axis=1, keepdims=True)

    # Rotate tangent 90 deg -> normal
    normals = np.zeros_like(tangents)
    normals[:, 0] = -tangents[:, 1]
    normals[:, 1] = tangents[:, 0]
    return normals

def make_offset(centerline, normals, d):
    """Return lateral offset d"""
    return centerline + d * normals

# Examples: centerline and offset
normals = compute_normals(centerline)
offsets = {
    "center": centerline,
    "d=-1.0": make_offset(centerline, normals, -1.0),
    "d=-0.5": make_offset(centerline, normals, -0.5),
    "d=+0.5": make_offset(centerline, normals, +0.5),
    "d=+1.0": make_offset(centerline, normals, +1.0),
}

# Load new image
img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
colors = {
    "center": (0, 0, 255),   # red
    "d=-1.0": (255, 0, 0),   # blue
    "d=-0.5": (200, 0, 200), # purple
    "d=+0.5": (0, 200, 200), # blue-green
    "d=+1.0": (0, 255, 0),   # green
}

for key, pts in offsets.items():
    for (x, y) in pts:
        u, v = world_to_pixel(x, y, origin, resolution, img.shape[0])
        cv2.circle(img_color, (u, v), 1, colors[key], -1)

cv2.imwrite(out_png_path, img_color)
