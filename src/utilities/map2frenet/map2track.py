import os
import cv2, yaml
import numpy as np
import pandas as pd
from skimage.morphology import skeletonize
import networkx as nx

# == User input ==
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/racetracks/"
map_name = "levine/levine_blacked_"

yaml_path = os.path.join(map_dir, map_name + "map.yaml")
png_path = os.path.join(map_dir, map_name + "map.png")
csv_path = os.path.join(map_dir, map_name + "track_center.csv")

track_spacing = 0.1

# 1. Load YAML
with open(yaml_path, 'r') as f:
    map_yaml = yaml.safe_load(f)

resolution = map_yaml["resolution"]
origin = map_yaml["origin"][:2]

# 2. Load map
img = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
free_space = (binary == 255).astype(np.uint8)

# 3. Extract skeleton
skeleton = skeletonize(free_space).astype(np.uint8)
points = np.argwhere(skeleton > 0)
h, w = free_space.shape

# 4. Convert skeleton to graph
G = nx.Graph()
for y, x in points:
    for dy in [-1,0,1]:
        for dx in [-1,0,1]:
            if dy==0 and dx==0: continue
            ny, nx_ = y+dy, x+dx
            if 0<=ny<h and 0<=nx_<w and skeleton[ny, nx_]>0:
                G.add_edge((y,x), (ny,nx_))

# 5. Longest cycle(loop)
cycles = nx.cycle_basis(G)
longest_cycle = max(cycles, key=len)

# Loop coordiantes
loop_coords = np.array(longest_cycle)

# 6. pixel to world frame
def pix2world(y,x,h):
    X = origin[0] + x*resolution
    Y = origin[1] + (h-y)*resolution
    return np.array([X,Y])

world_coords = np.array([pix2world(y,x,h) for y,x in loop_coords])

# 7. Resampling with "track_spacing"
sampled = [world_coords[0]]
acc = 0.0
for i in range(1,len(world_coords)):
    d = np.linalg.norm(world_coords[i]-world_coords[i-1])
    acc += d
    if acc>=track_spacing:
        sampled.append(world_coords[i])
        acc=0.0
sampled = np.array(sampled)

### Done: sampled == closed looped cycle.

# 8. tangent/normal + Get track_width with laycasting
def raycast(start_pix, direction, free_space, max_steps=500):
    y,x = start_pix
    steps=0
    while steps<max_steps:
        y+=direction[0]; x+=direction[1]
        if (y<0 or x<0 or y>=free_space.shape[0] or x>=free_space.shape[1]):
            break
        if free_space[int(y),int(x)]==0:
            return np.linalg.norm([y-start_pix[0], x-start_pix[1]])
        steps+=1
    return None

out=[]
for i in range(1,len(loop_coords)-1):
    y,x = loop_coords[i]
    prev = loop_coords[i-1]
    nxt = loop_coords[i+1]
    tangent = np.array([nxt[0]-prev[0], nxt[1]-prev[1]], dtype=float)
    tangent /= (np.linalg.norm(tangent)+1e-6)
    normal_left = np.array([-tangent[1], tangent[0]])
    normal_right = -normal_left

    left_pix = raycast((float(y),float(x)), normal_left, free_space)
    right_pix = raycast((float(y),float(x)), normal_right, free_space)
    if left_pix and right_pix:
        pos = pix2world(y,x,h)
        out.append([pos[0], pos[1], left_pix*resolution, right_pix*resolution])

df = pd.DataFrame(out, columns=["x","y","left_dist","right_dist"])
df.to_csv(csv_path, index=False)
print("Saved centerline loop:", len(out), "points")
