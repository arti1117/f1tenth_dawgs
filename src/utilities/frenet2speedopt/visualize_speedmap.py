
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import yaml, cv2
from PIL import Image

# == User input ==
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/maps/"
map_name = "icheon/icheon1009_"

yaml_path = os.path.join(map_dir, map_name + "map.yaml")
png_path = os.path.join(map_dir, map_name + "map.png")
csv_path = os.path.join(map_dir, map_name + "map_lippboyd_speedopted.csv")

def load_map(yaml_path="map.yaml"):
    with open(yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)

    resolution = map_info["resolution"]     # [m/pixel]
    origin = map_info["origin"]             # [x0, y0, yaw]

    img = Image.open(png_path)
    img = np.array(img)

    return img, resolution, origin

def world_to_map(x, y, resolution, origin, img_height):
    u = (x - origin[0]) / resolution
    v = img_height - (y - origin[1]) / resolution  # Y축 reverse
    return u, v

def visualize_path_on_map(csv_path="path_speedopted.csv", yaml_path="map.yaml"):
    # 경로 데이터 로드
    df = pd.read_csv(csv_path)
    x, y, v = df["x"], df["y"], df["v"]

    # 맵 불러오기
    img, resolution, origin = load_map(yaml_path)
    h, w = img.shape

    # 좌표 변환
    u, v_img = world_to_map(x, y, resolution, origin, h)

    # 시각화
    fig, ax = plt.subplots(figsize=(10,10))
    ax.imshow(img, cmap='gray', origin='upper')

    sc = ax.scatter(u, v_img, c=df["v"], cmap='jet', s=8)
    plt.colorbar(sc, label="Velocity [m/s]", ax=ax)

    ax.set_title("Velocity Profile on Map")
    plt.show()

if __name__ == "__main__":
    visualize_path_on_map(csv_path, yaml_path)
