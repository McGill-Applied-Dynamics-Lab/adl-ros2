import argparse
import pickle
from pathlib import Path

import numpy as np

# Defaults match the experiment ranges expected by pinch_main.py.
DEFAULT_N_SAMPLES = 400
AZIMUTH_RANGE_RAD = [-np.pi / 4, np.pi / 4]  # full revolution around the tube
NORMALIZED_HEIGHT_RANGE = [0.0, 1.0]  # scaled by TUBE_LENGTH in pinch_main.py
PINCH_DEPTH_RANGE_M = [0.0050, 0.0180]  # one-finger push (m); upper ~ TUBE_DIAMETER/2
PINCH_SPEED_RANGE_MPS = [0.0250, 0.100]  # gripper closing speed (m/s)

DEFAULT_OUTPUT = (
    Path(__file__).resolve().parent / "results" / "grids" / "pinch_params.pkl"
)


def generate_pinch_parameters(
    n_samples: int = DEFAULT_N_SAMPLES,
    output_path: Path = DEFAULT_OUTPUT,
    seed: int | None = None,
):
    """
    Generate randomized pinch parameters and save them as a pickle for pinch_main.py.

    Saved keys:
        azimuths            — angles around the tube (rad), 0..2*pi
        normalized_heights  — heights along the tube (unitless), 0..1; scaled by tube height in pinch_main.py
        pinch_depths        — gripper push depth per finger (m)
        pinch_speeds        — gripper closing speed (m/s)
        num_points          — number of samples
    """
    rng = np.random.default_rng(seed)

    azimuths = rng.uniform(AZIMUTH_RANGE_RAD[0], AZIMUTH_RANGE_RAD[1], n_samples)
    normalized_heights = rng.uniform(
        NORMALIZED_HEIGHT_RANGE[0], NORMALIZED_HEIGHT_RANGE[1], n_samples
    )
    pinch_depths = rng.uniform(
        PINCH_DEPTH_RANGE_M[0], PINCH_DEPTH_RANGE_M[1], n_samples
    )
    pinch_speeds = rng.uniform(
        PINCH_SPEED_RANGE_MPS[0], PINCH_SPEED_RANGE_MPS[1], n_samples
    )

    data = {
        "azimuths": azimuths,
        "normalized_heights": normalized_heights,
        "pinch_depths": pinch_depths,
        "pinch_speeds": pinch_speeds,
        "num_points": n_samples,
    }

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "wb") as f:
        pickle.dump(data, f)

    print(f"Generated {n_samples} pinch parameters.")
    print(
        f"  azimuths            in [{AZIMUTH_RANGE_RAD[0]:.3f}, {AZIMUTH_RANGE_RAD[1]:.3f}] rad"
    )
    print(
        f"  normalized_heights  in [{NORMALIZED_HEIGHT_RANGE[0]:.2f}, {NORMALIZED_HEIGHT_RANGE[1]:.2f}] (scaled by tube height in pinch_main.py)"
    )
    print(
        f"  pinch_depths        in [{PINCH_DEPTH_RANGE_M[0]:.3f}, {PINCH_DEPTH_RANGE_M[1]:.3f}] m"
    )
    print(
        f"  pinch_speeds        in [{PINCH_SPEED_RANGE_MPS[0]:.3f}, {PINCH_SPEED_RANGE_MPS[1]:.3f}] m/s"
    )
    print(f"Saved to: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate pinch_main.py parameter grid."
    )
    parser.add_argument("-n", "--n-samples", type=int, default=DEFAULT_N_SAMPLES)
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    generate_pinch_parameters(
        n_samples=args.n_samples, output_path=args.output, seed=args.seed
    )
