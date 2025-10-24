import os
import numpy as np
import matplotlib.pyplot as plt

# CSV 불러오기
script_dir = os.path.dirname(os.path.abspath(__file__))
table_dir = script_dir + "/../config/"
table_name = table_dir + "dawgs_lookup_table.csv"

data = np.loadtxt(table_name, delimiter=",")

# 속도 (첫 행, 첫 열 제외)
velocities = data[0, 1:]
# 스티어링 각도 (첫 열, 첫 행 제외)
steer_angles = data[1:, 0]
# 가속도 값 (내부 값)
accelerations = data[1:, 1:]

# 그리드 생성
V, S = np.meshgrid(velocities, steer_angles)

# 3D 플롯
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection="3d")

surf = ax.plot_surface(V, S, accelerations, cmap="viridis")

ax.set_xlabel("Velocity")
ax.set_ylabel("Steer Angle")
ax.set_zlabel("Acceleration")
ax.set_title("Lookup Table 3D Visualization")

fig.colorbar(surf, shrink=0.5, aspect=10)
plt.show()