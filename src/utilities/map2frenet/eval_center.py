import os
import cv2, yaml
import pandas as pd
import numpy as np
from scipy.spatial import cKDTree

# == User input ==
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/racetracks/"
map_name = "Shanghai/Shanghai_"

yaml_path = os.path.join(map_dir, map_name + "map.yaml")
png_path = os.path.join(map_dir, map_name + "map.png")

cand_csv_path = os.path.join(map_dir, map_name + "track_center.csv") # candidate centerline
cand2_csv_path = os.path.join(map_dir, map_name + "frenet_track.csv") # candidate2 centerline
ref_csv_path = os.path.join(map_dir, map_name + "centerline.csv") # reference centerline

# 1. Load YAML
with open(yaml_path, 'r') as f:
    map_yaml = yaml.safe_load(f)

resolution = map_yaml["resolution"]
origin = map_yaml["origin"][:2]

# 2. Load csv file
df_cand = pd.read_csv(cand_csv_path)
centerline_cand = df_cand.iloc[:, :2].to_numpy()
df_cand2 = pd.read_csv(cand2_csv_path)
centerline_cand2 = df_cand2.iloc[:, :2].to_numpy()
df_ref = pd.read_csv(ref_csv_path)
centerline_ref = df_ref.iloc[:, :2].to_numpy()

def world_to_pixel(x, y, origin, resolution, img_height):
    u = int((x - origin[0]) / resolution)
    v = int(img_height - (y - origin[1]) / resolution)
    return u, v


def align_centerlines(ref, cand):
    """
    ref: (N,2) centerline
    cand: (M,2) centerline
    → cand를 ref에 맞게 circular shift & flip 하여 정렬된 버전 반환
    """
    # Math the length
    N = len(ref)
    idx = np.linspace(0, len(cand)-1, N).astype(int)
    cand_resampled = cand[idx]

    # KDTree로 shift 최적화
    tree = cKDTree(ref)
    dists, indices = tree.query(cand_resampled)
    shift = indices[0]  # 첫 점 기준 shift

    cand_aligned = np.roll(cand_resampled, -shift, axis=0)

    # 방향 비교 (정방향 vs 역방향)
    error_forward = np.mean(np.linalg.norm(ref - cand_aligned, axis=1))
    error_reverse = np.mean(np.linalg.norm(ref - cand_aligned[::-1], axis=1))

    if error_reverse < error_forward:
        cand_aligned = cand_aligned[::-1]

    return cand_aligned

# 비교
cand_aligned = align_centerlines(centerline_ref, centerline_cand)
cand_aligned2 = align_centerlines(centerline_ref, centerline_cand2)
diff = np.linalg.norm(centerline_ref - cand_aligned, axis=1)
diff2 = np.linalg.norm(centerline_ref - cand_aligned2, axis=1)

print("Cand: track_center, ref: centerline")
print("Mean error:", diff.mean())
print("Max error:", diff.max())
print("Std error:", diff.std())
print("")
print("Cand: frenet_track, ref: centerline")
print("Mean error:", diff2.mean())
print("Max error:", diff2.max())
print("Std error:", diff2.std())
