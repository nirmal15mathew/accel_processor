
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from calculate_methods import *


def load_accel_data(csv_path):
    """
    Load accelerometer CSV and standardise column names.
    """
    df = pd.read_csv(csv_path)

    df = df.rename(columns={
        'Time (s)': 'time',
        'Linear Acceleration x (m/s^2)': 'ax',
        'Linear Acceleration y (m/s^2)': 'ay',
        'Linear Acceleration z (m/s^2)': 'az'
    })

    return df[['time', 'ax', 'ay', 'az']]


def plot_2d(px, py):
    plt.figure(figsize=(7, 6))

    # Trajectory
    plt.plot(px, py, label="Trajectory")

    # Start & End markers
    plt.scatter(px[0], py[0], c='green', s=60, label="Start")
    plt.scatter(px[-1], py[-1], c='red', s=60, label="End")

    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Estimated Device Path (XY)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()

def plot_3d(px, py, pz):
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    # Trajectory
    ax.plot(px, py, pz, label="Trajectory")

    # Start & End markers
    ax.scatter(px[0], py[0], pz[0], c='green', s=60, label="Start")
    ax.scatter(px[-1], py[-1], pz[-1], c='red', s=60, label="End")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Estimated 3D Device Path")

    ax.legend()
    plt.show()



def plot_trajectory(accel_csv_path):
    """
    High-level function:
    CSV -> trajectory -> plots
    """
    df = load_accel_data(accel_csv_path)

    time = df['time'].values
    ax = df['ax'].values
    ay = df['ay'].values
    az = df['az'].values

    px, py, pz = estimate_trajectory(time, ax, ay, az)
    mag, _ = compute_vector_displacement(time, ax, ay, az)
    print(mag)

    plot_2d(px, py)
    plot_3d(px, py, pz)
