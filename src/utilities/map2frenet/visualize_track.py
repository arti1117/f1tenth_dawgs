import os
import cv2, yaml
import pandas as pd
import numpy as np

# == User input ==
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/maps/"
map_name = "mohyun_1017/mohyun_1017_2"

yaml_path = os.path.join(map_dir, map_name + "map.yaml")
png_path = os.path.join(map_dir, map_name + "map.png")
csv_path = os.path.join(map_dir, map_name + "track_center.csv")
out_png_path = os.path.join(map_dir, map_name + "track_check.png")

ref_csv_path = os.path.join(map_dir, map_name + "centerline.csv")
out_ref_png_path = os.path.join(map_dir, map_name + "centerline_check.png")

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

# Load new image
img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# centerline = numpy array of shape (N,2)
for (x, y) in centerline:
    u, v = world_to_pixel(x, y, origin, resolution, img.shape[0])
    cv2.circle(img_color, (u, v), 2, (0, 0, 255), -1)  # red dots

cv2.imwrite(out_png_path, img_color)
