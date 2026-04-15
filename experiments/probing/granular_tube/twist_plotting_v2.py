import pickle
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# File path configuration
PROJECT_ROOT = Path(__file__).resolve().parent
RESULTS_FILE = PROJECT_ROOT / "results" / "RANDOM_TWIST.pkl"

def main():
    if not RESULTS_FILE.exists():
        print(f"Error: Could not find data file at {RESULTS_FILE}")
        return

    # Load the data
    print(f"Loading data from {RESULTS_FILE}...")
    with open(RESULTS_FILE, "rb") as f:
        data = pickle.load(f)

    num_experiments = len(data["target_angles"])
    print(f"Found {num_experiments} experiments. Plotting...")

    # Set up the figure with 3 subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=False)

    # Downsample factor for pressure
    DS = 10

    for i in range(num_experiments):
        # Time vectors (robot)
        t_fwd = np.array(data["ts_forward"][i])
        t_rev = np.array(data["ts_reverse"][i])
        t_rev_shifted = t_rev + t_fwd[-1]

        # Robot data
        ang_fwd = np.degrees(data["joint7_angles_forward"][i])
        ang_rev = np.degrees(data["joint7_angles_reverse"][i])

        # Torques (z-direction only)
        trq_fwd = np.array(data["torques_forward"][i])[:, 2]
        trq_rev = np.array(data["torques_reverse"][i])[:, 2]

        # Pressures
        p_fwd = data["pressures_forward"][i]
        p_rev = data["pressures_reverse"][i]

        # Create time vectors based on Teensy elapsed time (dt_micros)
        t_press_fwd = np.linspace(0, p_fwd["dt_micros"] / 1e6, len(p_fwd["sensor1"]))
        t_press_rev = np.linspace(0, p_rev["dt_micros"] / 1e6, len(p_rev["sensor1"])) + t_fwd[-1]

        # Naively downsample pressure data by factor of DS (10)
        t_press_fwd_ds = t_press_fwd[::DS]
        s1_fwd_ds = p_fwd["sensor1"][::DS]
        s2_fwd_ds = p_fwd["sensor2"][::DS]

        t_press_rev_ds = t_press_rev[::DS]
        s1_rev_ds = p_rev["sensor1"][::DS]
        s2_rev_ds = p_rev["sensor2"][::DS]

        # Plotting
        color = f"C{i % 10}" # Cycle through standard matplotlib colors

        # Subplot 1: Angles
        axs[0].plot(t_fwd, ang_fwd, color=color, alpha=0.7)
        axs[0].plot(t_rev_shifted, ang_rev, color=color, alpha=0.7)

        # Subplot 2: Z-Axis Torques
        axs[1].plot(t_fwd, trq_fwd, color=color, alpha=0.7)
        axs[1].plot(t_rev_shifted, trq_rev, color=color, alpha=0.7)

        # Subplot 3: Pressures (Sensor 1 in solid, Sensor 2 in dashed)
        axs[2].plot(t_press_fwd_ds, s1_fwd_ds, color=color, linestyle='-', alpha=0.7)
        axs[2].plot(t_press_rev_ds, s1_rev_ds, color=color, linestyle='-', alpha=0.7)

        axs[2].plot(t_press_fwd_ds, s2_fwd_ds, color=color, linestyle='--', alpha=0.5)
        axs[2].plot(t_press_rev_ds, s2_rev_ds, color=color, linestyle='--', alpha=0.5)

    # Formatting
    axs[0].set_title("Wrist Twist Angle")
    axs[0].set_ylabel(r"$\phi$ (deg.)")
    axs[0].grid(True, linestyle='--', alpha=0.6)

    axs[1].set_title("End Effector Torque (Z-Axis)")
    axs[1].set_ylabel("Torque (Nm)")
    axs[1].grid(True, linestyle='--', alpha=0.6)

    axs[2].set_title(f"Pressure Sensor Signals (10x Downsampled)")
    axs[2].set_ylabel("ADC Value")
    axs[2].set_xlabel("Time (s)")
    axs[2].grid(True, linestyle='--', alpha=0.6)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()