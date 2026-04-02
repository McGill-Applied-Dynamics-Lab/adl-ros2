import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import pickle


def generate_grid(x_extent, y_extent, nx, ny, xdir=1, ydir=1):
    """Generate training and testing grid points within a rectangular area."""
    if nx < 1 or ny < 1:
        raise ValueError("nx and ny must be >= 1")

    # Find grid points
    x_train = np.linspace(0.0, xdir * x_extent, nx) if nx > 1 else np.array([0.0])
    y_train = np.linspace(0.0, ydir * y_extent, ny) if ny > 1 else np.array([0.0])

    dx = xdir * x_extent / (nx - 1)
    dy = ydir * y_extent / (ny - 1)

    x_centers = x_train[:-1] + dx / 2.0
    y_centers = y_train[:-1] + dy / 2.0

    # Create train/test matrices
    # Create meshgrid and stack as (2, N) arrays
    x_train, y_train = np.meshgrid(x_train, y_train)
    x_centers, y_centers = np.meshgrid(x_centers, y_centers)
    train = np.vstack([x_train.ravel(), y_train.ravel()])
    test = np.vstack([x_centers.ravel(), y_centers.ravel()])

    return train, test


def fetch_landmarks(landmark_file: str, to_fetch: list):
    """Fetch landmark coordinates from file."""
    with open(landmark_file) as f:
        values = {}
        for line in f:
            if ":" in line:
                key, val = line.strip().split(":")
                if key.strip() in to_fetch:
                    values[key.strip()] = float(val.strip())
    return values


# ---- Example usage ----
if __name__ == "__main__":
    """Generate waveguide gripper probing grid (test/train) and save to file."""
    PROJECT_ROOT = Path(__file__).resolve().parent
    LANDMARK_FILE = (
        PROJECT_ROOT/ "results" / "grids" / "landmarks.txt"
    )
    PROBE_DIAMETER = 0.0  # (m)
    GRIPPER_WIDTH = 0.0320  # (m)
    GRIPPER_LENGTH = 0.1250  # (m)
    X_SHIFT = -0.0070  # from CAD (m)
    Y_SHIFT = 0.0050  # from CAD (m)
    X_EXTENT = GRIPPER_WIDTH  # (m)
    Y_EXTENT = GRIPPER_LENGTH  # (m)

    # Even spacing along x and y directions
    dl = 15 / 1000  # points per meter
    nx = int(X_EXTENT / dl)
    ny = int(Y_EXTENT / dl)

    grids = {}

    train_, test_ = generate_grid(
        x_extent=X_EXTENT, y_extent=Y_EXTENT, nx=nx, ny=ny, xdir=-1, ydir=1
    )  # generate rectangular grid

    # Store gripper frame (before landmark offset)
    grids["GRIPPER_FRAME"] = {"train": train_.T, "test": test_.T}

    print(f"Length of train set: {len(train_.T)}")
    print(f"Length of test set: {len(test_.T)}")

    # Apply shifts
    train_[0, :] += X_SHIFT
    train_[1, :] += Y_SHIFT
    test_[0, :] += X_SHIFT
    test_[1, :] += Y_SHIFT

    # Apply landmark offset
    landmarks = fetch_landmarks(LANDMARK_FILE, ["x", "y", "z"])
    train_[0, :] += landmarks["x"]
    train_[1, :] += landmarks["y"]
    test_[0, :] += landmarks["x"]
    test_[1, :] += landmarks["y"]

    # Store world frame (after landmark offset) - transpose to (N, 2) shape
    train = train_.T
    test = test_.T
    grids["WORLD_FRAME"] = {"train": train, "test": test}

    # Save grid using pickle
    grid_file = PROJECT_ROOT / "results" / "grids" / f"grids.pkl"
    with open(grid_file, "wb") as f:
        pickle.dump(grids, f)
    print("Grid saved to:", grid_file)

    # Plot grids
    plt.title(f"Train points: {len(train)}, Test points: {len(test)}")
    plt.scatter(train[:, 0], train[:, 1], color="b", label="Train (World Frame)")
    plt.scatter(test[:, 0], test[:, 1], color="g", label="Test (World Frame)")
    plt.scatter(landmarks["x"], landmarks["y"], marker="x", color="r", s=100, label="Landmark")
    plt.axis("equal")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.show()
