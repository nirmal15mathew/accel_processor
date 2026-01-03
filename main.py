import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ----------------------------
# Load CSV
# ----------------------------

ACCEL_PATH = "./acceleration_data/Raw Data.csv"
df = pd.read_csv(ACCEL_PATH)

# Rename columns for convenience
df = df.rename(columns={
    'Time (s)': 'time',
    'Linear Acceleration x (m/s^2)': 'ax',
    'Linear Acceleration y (m/s^2)': 'ay',
    'Linear Acceleration z (m/s^2)': 'az'
})

# ----------------------------
# Time handling
# ----------------------------
time = df['time'].values

# Compute time differences
dt = np.diff(time, prepend=time[0])

# ----------------------------
# Acceleration arrays
# ----------------------------
ax = df['ax'].values
ay = df['ay'].values
az = df['az'].values

# ----------------------------
# Integrate acceleration → velocity
# v(t) = ∫ a(t) dt
# ----------------------------
vx = np.cumsum(ax * dt)
vy = np.cumsum(ay * dt)
vz = np.cumsum(az * dt)

# ----------------------------
# Integrate velocity → position
# x(t) = ∫ v(t) dt
# ----------------------------
px = np.cumsum(vx * dt)
py = np.cumsum(vy * dt)
pz = np.cumsum(vz * dt)

# ----------------------------
# Plot 2D trajectory (XY plane)
# ----------------------------
plt.figure(figsize=(7, 6))
plt.plot(px, py, label="Device Path")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Estimated Device Trajectory (XY)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()

# ----------------------------
# Optional: 3D trajectory
# ----------------------------
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(8, 6))
ax3d = fig.add_subplot(111, projection='3d')
ax3d.plot(px, py, pz)
ax3d.set_xlabel("X (m)")
ax3d.set_ylabel("Y (m)")
ax3d.set_zlabel("Z (m)")
ax3d.set_title("Estimated 3D Device Path")
plt.show()
