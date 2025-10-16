
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ----- User Inputs -----
script_dir = os.path.dirname(os.path.abspath(__file__))
map_dir = script_dir + "/../../peripheral/maps/"
map_name = "icheon/icheon1009_map"

ref_name = "lippboyd_speedopt.csv"
cand_name = "forback_speedopted.csv"
ref_path = os.path.join(map_dir, map_name + ref_name) # reference
cand_path = os.path.join(map_dir, map_name + cand_name) # candidates


# 2. Load velocity
def load_path(ref, cand):
    df_ref = pd.read_csv(ref)
    if {'v'}.issubset(df_ref.columns):
        v1 = df_ref['v'].to_numpy()

    df_cand = pd.read_csv(cand)
    if {'v'}.issubset(df_cand.columns):
        v2 = df_cand['v'].to_numpy()

    df = pd.DataFrame({
        'v1': v1,
        'v2': v2
    })
    return df


# 평균과 표준편차 계산
df = load_path(ref_path, cand_path)
mean_v1 = df['v1'].mean()
std_v1 = df['v1'].std()
mean_v2 = df['v2'].mean()
std_v2 = df['v2'].std()

print(f"{ref_name} mean: {mean_v1:.3f}, std: {std_v1:.3f}")
print(f"{cand_name} mean: {mean_v2:.3f}, std: {std_v2:.3f}")

# Plot
plt.figure(figsize=(8,5))
plt.plot(df['v1'], label=ref_name)
plt.plot(df['v2'], label=cand_name)
plt.xlabel('Index')
plt.ylabel('Velocity')
plt.title('Velocity Profiles Comparison')
plt.legend()
plt.grid(True)
plt.show()