#!/usr/bin/env python3
import pickle
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def generate_actuator_points(
    actuator_length: float,
    first_point_xy: np.ndarray,
    x_column_spacing: float,
    y_point_spacing: float,
    num_columns: int,
) -> np.ndarray:
    x_positions = first_point_xy[0] + np.arange(num_columns, dtype=float) * x_column_spacing
    y_positions = np.arange(first_point_xy[1], actuator_length, y_point_spacing, dtype=float)

    points: list[list[float]] = []
    for x in x_positions:
        for y in y_positions:
            points.append([x, y])
    return np.asarray(points, dtype=float)


def generate_grid(x_extent, y_extent, nx, ny, xdir=1, ydir=1):
    if nx < 1 or ny < 1:
        raise ValueError("nx and ny must be >= 1")

    x_train = np.linspace(0.0, xdir * x_extent, nx) if nx > 1 else np.array([0.0])
    y_train = np.linspace(0.0, ydir * y_extent, ny) if ny > 1 else np.array([0.0])

    dx = xdir * x_extent / (nx - 1)
    dy = ydir * y_extent / (ny - 1)

    x_centers = x_train[:-1] + dx / 2.0
    y_centers = y_train[:-1] + dy / 2.0

    x_train, y_train = np.meshgrid(x_train, y_train)
    x_centers, y_centers = np.meshgrid(x_centers, y_centers)
    train = np.vstack([x_train.ravel(), y_train.ravel()])
    test = np.vstack([x_centers.ravel(), y_centers.ravel()])
    return train, test


def fetch_landmarks(landmark_file: str | Path, to_fetch: list[str]):
    values = {}
    with open(landmark_file, "r", encoding="utf-8") as f:
        for line in f:
            if ":" not in line:
                continue
            key, val = line.strip().split(":", maxsplit=1)
            key = key.strip()
            if key in to_fetch:
                values[key] = float(val.strip())
    return values


if __name__ == "__main__":
    project_root = Path(__file__).resolve().parent
    results_root = project_root / "results" / "grids"
    results_root.mkdir(parents=True, exist_ok=True)

    landmark_file = results_root / "landmarks.txt"
    gripper_width = 0.0320
    gripper_length = 0.1250
    x_shift = 0.0
    y_shift = -0.0050
    first_point_xy = np.array([(4.381 + 2.5) / 1000.0, 3.0 / 1000.0], dtype=float)
    x_column_spacing = 8.096 / 1000.0
    y_point_spacing = 5.0 / 1000.0
    num_columns = 4

    train = generate_actuator_points(
        actuator_length=gripper_length,
        first_point_xy=first_point_xy,
        x_column_spacing=x_column_spacing,
        y_point_spacing=y_point_spacing,
        num_columns=num_columns,
    )
    dl = 15 / 1000
    nx = int(gripper_width / dl)
    ny = int(gripper_length / dl)
    _, test_ = generate_grid(
        x_extent=gripper_width,
        y_extent=gripper_length,
        nx=nx,
        ny=ny,
        xdir=-1,
        ydir=-1,
    )
    test = test_.T

    grids = {"GRIPPER_FRAME": {"train": train.copy(), "test": test.copy()}}

    train_world = train.copy()
    test_world = test.copy()
    train_world[:, 0] = -(train_world[:, 0] + x_shift)
    train_world[:, 1] = -(train_world[:, 1] + y_shift)
    test_world[:, 0] += x_shift
    test_world[:, 1] += y_shift

    landmarks = fetch_landmarks(landmark_file, ["x", "y", "z"])
    train_world[:, 0] += landmarks["x"]
    train_world[:, 1] += landmarks["y"]
    test_world[:, 0] += landmarks["x"]
    test_world[:, 1] += landmarks["y"]

    grids["WORLD_FRAME"] = {"train": train_world, "test": test_world}

    grid_file = results_root / "grids.pkl"
    with open(grid_file, "wb") as f:
        pickle.dump(grids, f)

    print(f"Grid saved to: {grid_file}")
    plt.title(f"Train points: {len(train_world)}, Test points: {len(test)}")
    plt.scatter(-train_world[:, 0], -train_world[:, 1], color="b", label="Train (World Frame)")
    plt.scatter(-test_world[:, 0], -test_world[:, 1], color="g", label="Test (World Frame)")
    plt.scatter(-landmarks["x"], -landmarks["y"], marker="x", color="r", s=100, label="Landmark")
    plt.axis("equal")
    plt.xlabel("-X (m)")
    plt.ylabel("-Y (m)")
    plt.legend()
    plt.show()
