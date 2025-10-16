import os
import cv2
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# == User input ==
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/racetracks/"
map_name = "levine/levine_blacked_"

yaml_path = os.path.join(map_dir, map_name + "map.yaml")
png_path = os.path.join(map_dir, map_name + "map.png")
csv_path = os.path.join(map_dir, map_name + "track_center_rev.csv")
out_img_path = os.path.join(map_dir, map_name + "track_center_rev_vis.png")

reverse_option = False # True or False

# 1. Load yaml
with open(yaml_path, 'r') as f:
    map_yaml = yaml.safe_load(f)
resolution = map_yaml["resolution"]
origin = map_yaml["origin"][:2]

# 2. Load background map
img = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
h, w = img.shape

# 3. Load centerline csv
df = pd.read_csv(csv_path)
xs, ys = df["x"].to_numpy(), df["y"].to_numpy()

# 4. World → pixel 변환
def world2pix(X, Y, h):
    x = (X - origin[0]) / resolution
    y = h - (Y - origin[1]) / resolution
    return int(x), int(y)

pts_pix = np.array([world2pix(X, Y, h) for X, Y in zip(xs, ys)])

# 5. 인덱스 기반 그라데이션
idx_norm = np.linspace(0, 1, len(pts_pix))  # 0 ~ 1
cmap = plt.cm.jet  # jet colormap (파랑→초록→노랑→빨강)

# 6. 시각화
plt.figure(figsize=(8,8))
plt.imshow(img, cmap="gray", origin="upper")

for i in range(len(pts_pix)-1):
    p1, p2 = pts_pix[i], pts_pix[i+1]
    color = cmap(idx_norm[i])
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color, linewidth=0.5)

# 7. 80번째마다 index 표시
for i in range(0, len(pts_pix), 80):
    x, y = pts_pix[i]
    plt.text(x, y, str(i), color="red", fontsize=2, ha="center")

plt.axis("equal")
plt.gca().invert_yaxis()  # 이미지 좌표 맞추기
plt.title("Track Center with Direction Gradient")
plt.savefig(out_img_path, dpi=300)
plt.close()
print("Saved visualization:", out_img_path)

# 8. 필요시 방향 반전
def reverse_csv(in_path, out_path):
    df = pd.read_csv(in_path)
    df = df[::-1].reset_index(drop=True)
    df.to_csv(out_path, index=False)
    print("Reversed csv saved:", out_path)

if reverse_option:
    reverse_csv(csv_path, csv_path.replace(".csv", "_rev.csv"))

