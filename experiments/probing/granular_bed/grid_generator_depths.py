import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import pickle

"""
Uniform grid generation + depths.
"""


def generate_grid(x_extent, y_extent, nx, ny):
    """Generate training and testing grid points within a rectangular area."""
    if nx < 1 or ny < 1:
        raise ValueError("nx and ny must be >= 1")

    # Find grid points
    x_train = np.linspace(0.0, x_extent, nx) if nx > 1 else np.array([0.0])
    y_train = np.linspace(0.0, y_extent, ny) if ny > 1 else np.array([0.0])

    dx = x_extent / (nx - 1)
    dy = y_extent / (ny - 1)

    x_centers = x_train[:-1] + dx / 2.0
    y_centers = y_train[:-1] + dy / 2.0

    # Create train/test matrices
    # Create meshgrid and stack as (2, N) arrays
    x_train, y_train = np.meshgrid(x_train, y_train)
    x_centers, y_centers = np.meshgrid(x_centers, y_centers)
    train = np.vstack([x_train.ravel(), y_train.ravel()])
    test = np.vstack([x_centers.ravel(), y_centers.ravel()])

    return train, test


def find_rotation(landmark_file: str):
    """Find rotation matrix from landmark coordinates."""
    with open(landmark_file) as f:
        values = {}
        for line in f:
            if ":" in line:
                key, val = line.strip().split(":")
                values[key.strip()] = float(val.strip())

    # Create list of lists in desired order
    pbl = np.array([values["xbl"], values["ybl"]])
    pbr = np.array([values["xbr"], values["ybr"]])
    pdiff = pbr - pbl  # vector from bottom left to bottom right
    rot_angle = np.angle(pdiff[0] + 1j * pdiff[1])  # rotation angle in XY plane
    rot_matrix = np.array(
        [
            [np.cos(rot_angle), -np.sin(rot_angle)],
            [np.sin(rot_angle), np.cos(rot_angle)],
        ]
    )  # 2-D rotation matrix

    return [pbl, pbr], rot_matrix


# ---- Example usage ----
if __name__ == "__main__":
    PROJECT_ROOT = Path(__file__).resolve().parent
    LANDMARK_FILE = PROJECT_ROOT / "results" / "grids" / "landmarks.txt"
    # Check if landmark file exists
    if not LANDMARK_FILE.exists():
        raise FileNotFoundError(f"Landmark file not found: {LANDMARK_FILE}")
    PROBE_DIAMETER = 0.016  # (m)
    ADD_OFFSET = 0.005 # (m)
    X_SHIFT = 0.010 + PROBE_DIAMETER / 2 + ADD_OFFSET  # (m)
    Y_SHIFT = 0.010 + PROBE_DIAMETER / 2 + ADD_OFFSET  # (m)
    X_EXTENT = 0.10 - PROBE_DIAMETER - 2*ADD_OFFSET # (m)
    Y_EXTENT = 0.10 - PROBE_DIAMETER - 2*ADD_OFFSET # (m)
    grids = {}  # dictionary to hold grids

    # ***Add depths***
    DEPTHS = np.linspace(5e-3, 3e-2, 5) # plunge depths

    train_, test_ = generate_grid(
        x_extent=X_EXTENT, y_extent=Y_EXTENT, nx=2, ny=2
    )  # generate rectangular grid
    temp_train = train_.copy()
    temp_train[0, :] += PROBE_DIAMETER/2 + ADD_OFFSET
    temp_train[1, :] += PROBE_DIAMETER/2 + ADD_OFFSET
    temp_test = test_.copy()
    temp_test[0, :] += PROBE_DIAMETER/2 + ADD_OFFSET
    temp_test[1, :] += PROBE_DIAMETER/2 + ADD_OFFSET
    temp_train = temp_train.T
    temp_test = temp_test.T
    grids["SENSOR_FRAME"] = {"train": temp_train, "test": temp_test}

    # Apply shifts
    train_[0, :] += X_SHIFT
    train_[1, :] += Y_SHIFT
    test_[0, :] += X_SHIFT
    test_[1, :] += Y_SHIFT

    landmarks, rot_matrix = find_rotation(landmark_file=LANDMARK_FILE)
    train = rot_matrix @ train_
    test = rot_matrix @ test_
    # Apply landmark offset
    train[0, :] += landmarks[0][0]
    train[1, :] += landmarks[0][1]
    test[0, :] += landmarks[0][0]
    test[1, :] += landmarks[0][1]
    # Transpose to (N, 2) shape
    train = train.T
    test = test.T
    grids["WORLD_FRAME"] = {"train": train, "test": test}

    # Repeat and randomize points
    RANDOM = False
    COPIES = len(DEPTHS)
    temp_train_rep = np.tile(temp_train, (COPIES, 1))
    temp_test_rep = np.tile(temp_test, (COPIES, 1))
    train_rep = np.tile(train, (COPIES, 1))
    test_rep = np.tile(test, (COPIES, 1))

    # Update grids with repeated points and append depths
    n_train = train.shape[0]
    n_test = test.shape[0]
    depths_train_expand = np.repeat(DEPTHS, n_train)[:, None]
    depths_test_expand = np.repeat(DEPTHS, n_test)[:, None]
    grids["SENSOR_FRAME"]["train"] = np.hstack((temp_train_rep, depths_train_expand))
    grids["SENSOR_FRAME"]["test"] = np.hstack((temp_test_rep, depths_test_expand))
    grids["WORLD_FRAME"]["train"] = np.hstack((train_rep, depths_train_expand))
    grids["WORLD_FRAME"]["test"] = np.hstack((test_rep, depths_test_expand))

    # Save
    with open(PROJECT_ROOT / "results" / "grids" / "grids.pkl", "wb") as f:
        pickle.dump(grids, f)

    # Create plot in world frame
    fig, ax = plt.subplots()
    ax.set_title(f"Train points: {len(grids['WORLD_FRAME']['train'])}, Test points: {len(grids['WORLD_FRAME']['test'])}")
    ax.scatter(grids["WORLD_FRAME"]['train'][:,0],
               grids["WORLD_FRAME"]['train'][:,1],
               color="b", label="Train (World Frame)")
    ax.scatter(grids["WORLD_FRAME"]['test'][:,0],
                grids["WORLD_FRAME"]['test'][:,1],
                color="g", label="Test (World Frame)")
    plt.scatter(landmarks[0][0], landmarks[0][1], marker="x", color="r", s=100)
    plt.scatter(landmarks[1][0], landmarks[1][1], marker="x", color="r", s=100)
    plt.plot(
        [landmarks[0][0], landmarks[1][0]],
        [landmarks[0][1], landmarks[1][1]],
        "r-",
        label="Landmarks",
    )
    plt.axis("equal")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.show()

    # Create plot in sensor frame
    plt.title(f"Train points: {len(grids['SENSOR_FRAME']['train'])}, Test points: {len(grids['SENSOR_FRAME']['test'])}")
    plt.scatter(grids['SENSOR_FRAME']['train'][:,0], grids["SENSOR_FRAME"]['train'][:,1], color="b", label="Train (Sensor Frame)")
    plt.scatter(grids['SENSOR_FRAME']['test'][:,0], grids["SENSOR_FRAME"]['test'][:,1], color="g", label="Test (Sensor Frame)")
    plt.xlim(0, 0.1)
    plt.ylim(0, 0.1)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.show()
