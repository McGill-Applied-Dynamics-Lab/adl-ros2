#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import pickle
import pandas as pd

from ament_index_python.packages import get_package_share_directory


def bootstrap_data(df: pd.DataFrame) -> float:
    """
    Bootstrap the data to get a mean and standard deviation of the delay.
    """
    num_bootstrap_samples = 10_000
    bootstrap_means = []

    for _ in range(num_bootstrap_samples):
        sample = df["latency"].sample(n=len(df), replace=True)
        bootstrap_means.append(sample.mean())

    # # Plotting the histogram
    # plt.hist(bootstrap_means, bins=30, edgecolor="k")
    # plt.xlabel("Bootstrapped Mean Latency")
    # plt.ylabel("Frequency")
    # plt.title("Bootstrap Distribution of the Mean")
    # plt.show()

    # # Calculating mean and standard deviation
    # mean_delay = np.mean(bootstrap_means)
    # std_delay = 0.0

    return bootstrap_means


def plot_bootstrap_data(data_file):
    try:
        # Load bootstrap data
        dir_path = Path(__file__).parent
        with open(dir_path / data_file, "rb") as f:
            test_data = pickle.load(f)

        package_share_path = Path(get_package_share_directory("network_sim"))
        csv_file_path = package_share_path / "data/5G-data.csv"
        data_5g = pd.read_csv(csv_file_path)

        # Bootstrap data
        bootstrap_means = bootstrap_data(data_5g)

        # bootstrap_means = test_data["bootstrap_means"]
        mean_delay = test_data["node_mean_delay"]

        # Create the plot
        plt.figure(figsize=(10, 6))
        plt.hist(bootstrap_means, bins=30, edgecolor="k")
        plt.xlabel("Bootstrapped Mean Latency")
        plt.ylabel("Frequency")
        plt.title("Bootstrap Distribution of the Mean")

        plt.axvline(mean_delay, color="red", linestyle="--", label="Mean Delay")
        plt.legend()

        # Save plot to file
        plot_path = dir_path / "bootstrap_histogram.png"
        plt.savefig(plot_path)
        print(f"Plot saved to {plot_path}")

        # Show the plot
        plt.show(block=True)

    except Exception as e:
        print(f"Error plotting bootstrap data: {e}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 plot_bootstrap.py <data_file.pkl>")
        print('Defaulting to "bootstrap_test_data.pkl"')
        sys.argv.append("bootstrap_test_data.pkl")

    plot_bootstrap_data(sys.argv[1])
