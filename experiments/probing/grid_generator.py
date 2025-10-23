import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def make_probe_points(x_extent, y_extent, nx, ny):
    """
    Create train/test probing grids.

    Train:
        - Grid starts at (0, 0)
        - nx points across x in [0, x_extent]
        - ny points across y in [0, y_extent]
    Test:
        - Grid shifted by half a step in both x and y
        - Stays within [0, x_extent] x [0, y_extent]
        - Shares no coordinates with train points

    Parameters:
    x_extent, y_extent : float
        Max extents along x and y (inclusive for the train grid endpoints).
    nx, ny : int
        Number of grid points along x and y for the train grid (>=1).

    Returns:
    probe_points : dict
        {
          'train': np.ndarray of shape (nx*ny, 2),
          'test' : np.ndarray of shape (nx*ny, 2)
        }
    """
    if nx < 1 or ny < 1:
        raise ValueError("nx and ny must be >= 1")

    # Train grid
    x_train = np.linspace(0.0, x_extent, nx) if nx > 1 else np.array([0.0])
    y_train = np.linspace(0.0, y_extent, ny) if ny > 1 else np.array([0.0])
    XX_train, YY_train = np.meshgrid(x_train, y_train, indexing="xy")
    train = np.column_stack([XX_train.ravel(), YY_train.ravel()])

    dx = x_extent / (nx - 1)
    dy = y_extent / (ny - 1)

    x_centers = x_train[:-1] + dx / 2.0
    y_centers = y_train[:-1] + dy / 2.0

    # Test grid
    XX_test, YY_test = np.meshgrid(x_centers, y_centers, indexing="xy")
    test = np.column_stack([XX_test.ravel(), YY_test.ravel()])

    return train, test


# ---- Example usage ----
if __name__ == "__main__":
    PROJECT_ROOT = Path(__file__).resolve().parent  # or Path.cwd()
    print(PROJECT_ROOT)
    base_loc = PROJECT_ROOT / "grids"

    train, test = make_probe_points(x_extent=0.06, y_extent=0.06, nx=2, ny=2)
    np.save(Path(base_loc) / r"train_grid.npy", train)
    np.save(Path(base_loc) / r"test_grid.npy", test)
    print(len(train), len(test))
    plt.scatter(train[:, 0], train[:, 1])
    plt.scatter(test[:, 0], test[:, 1])
    plt.show()
