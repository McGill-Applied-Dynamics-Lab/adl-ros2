import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import pickle

def generate_grid(x_extent, y_extent, nx, ny, xdir=1, ydir=1):
    """ Generate training and testing grid points within a rectangular area. """
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
    train = np.row_stack([x_train.ravel(), y_train.ravel()])
    test = np.row_stack([x_centers.ravel(), y_centers.ravel()])

    return train, test

def fetch_landmarks(landmark_file: str, to_fetch: list):
    """ Fetch landmark coordinates from file. """
    with open(landmark_file) as f:
        values = {}
        for line in f:
            if ':' in line:
                key, val = line.strip().split(':')
                if key.strip() in to_fetch:
                    values[key.strip()] = float(val.strip())

    return values

# ---- Example usage ----
if __name__ == "__main__":
    """ Generate waveguide gripper probing grid (test/train) and save to file. """
    PROJECT_ROOT = Path(__file__).resolve().parent
    LANDMARK_FILE = PROJECT_ROOT / 'results' / 'grids' / "waveguide_gripper_landmarks.txt"
    PROBE_DIAMETER = 0.0 # (m)
    GRIPPER_WIDTH = 0.0320 # (m)
    GRIPPER_LENGTH = 0.1250 # (m)
    X_SHIFT =  0.0070 # from CAD (m)
    Y_SHIFT =  -0.0050 # from CAD (m)
    X_EXTENT = GRIPPER_WIDTH # (m)
    Y_EXTENT = GRIPPER_LENGTH # (m)

    # Even spacing along x and y directions
    dl = 5 / 1000 # points per meter
    nx = int(X_EXTENT / dl)
    ny = int(Y_EXTENT / dl)

    grid = {}
    # Save parameters
    grid['dl'] = dl
    grid['nx'] = nx
    grid['ny'] = ny

    train, test = generate_grid(x_extent=X_EXTENT, y_extent=Y_EXTENT, nx=nx, ny=ny, xdir=1, ydir=-1) # generate rectangular grid
    grid['train_gripper_frame'] = train.T
    grid['test_gripper_frame'] = test.T
    print(f"Length of train set: {len(train)}")
    print(f"Length of test set: {len(test)}")

    # Apply shifts
    train[0, :] += X_SHIFT
    train[1, :] += Y_SHIFT
    test[0, :] += X_SHIFT
    test[1, :] += Y_SHIFT
    # Apply landmark offset
    landmarks = fetch_landmarks(LANDMARK_FILE, ['x', 'y', 'z'])
    train[0, :] += landmarks['x']
    train[1, :] += landmarks['y']
    test[0, :] += landmarks['x']
    test[1, :] += landmarks['y']

    grid['train_world_frame'] = train.T
    grid['test_world_frame'] = test.T

    # Save grid using pickle
    grid_file = PROJECT_ROOT / "results" / "grids" / f"GRIPPER_GRID.pkl"
    with open(grid_file, "wb") as f:
        pickle.dump(grid, f)
    print("Grid saved to:", grid_file)

    # Plot grids
    plt.scatter(train[0, :], train[1, :], label='Train')
    plt.scatter(test[0, :], test[1, :], label='Test')
    plt.scatter(landmarks['x'], landmarks['y'], marker='x', color='k', s=50)
    plt.axis('equal')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.show()
