import zipfile
import shutil
from pathlib import Path
from datetime import datetime

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from plot_methods import plot_trajectory


def extract_zip(zip_path, extract_to):
    extract_to.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(extract_to)

def generate_run_name(meta_dir):
    device_file = meta_dir / "device.csv"
    time_file = meta_dir / "time.csv"

    device_name = "unknown_device"
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    if device_file.exists():
        df = pd.read_csv(device_file)
        device_name = df.iloc[0, 0]

    if time_file.exists():
        df = pd.read_csv(time_file)
        timestamp = str(df.iloc[0, 0]).replace(":", "-").replace(" ", "_")

    return f"device_{device_name}_{timestamp}"

def reorganise_data(extracted_dir, processed_root):
    meta_dir = extracted_dir / "meta"
    raw_data_file = extracted_dir / "Raw Data.csv"

    run_name = generate_run_name(meta_dir)
    run_dir = processed_root / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    shutil.move(str(raw_data_file), run_dir / "accel.csv")

    for meta_file in meta_dir.glob("*.csv"):
        shutil.move(str(meta_file), run_dir / meta_file.name)

    return run_dir




def main(zip_path, plot_data_path=None):
    zip_path = Path(zip_path)
    work_dir = Path("workspace")
    processed_root = Path("processed")

    extract_dir = work_dir / zip_path.stem
    extract_zip(zip_path, extract_dir)

    run_dir = reorganise_data(extract_dir, processed_root)

    if plot_data_path is None:
        plot_data_path = run_dir / "accel.csv"

    plot_trajectory(plot_data_path)


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python process_and_plot.py <zip_path> [accel_csv_path]")
        sys.exit(1)

    zip_path = sys.argv[1]
    accel_path = sys.argv[2] if len(sys.argv) > 2 else None

    main(zip_path, accel_path)