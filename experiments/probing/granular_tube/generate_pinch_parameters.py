import numpy as np
import pickle
from pathlib import Path

def generate_pinch_parameters(n_samples: int = 1500, filename: str = "PINCH_PARAMS.pkl"):
    """
    Generates n random angles [0, 2*pi) and normalized heights [0, 1].
    Saves them into a pickle file to be used by pinch_random.py.
    """
    # Random angles from 0 to 2*pi
    azimuths = np.random.uniform(0, 2 * np.pi, n_samples)
    
    # Random normalized heights from 0.0 to 1.0
    normalized_heights = np.random.uniform(0, 1.0, n_samples)
    
    data = {
        "azimuths": azimuths.tolist(),
        "normalized_heights": normalized_heights.tolist(),
        "n_samples": n_samples
    }
    
    save_dir = Path(__file__).resolve().parent / "results"
    save_dir.mkdir(parents=True, exist_ok=True)
    
    file_path = save_dir / filename
    with open(file_path, "wb") as f:
        pickle.dump(data, f)
        
    print(f"Generated {n_samples} pinch parameters.")
    print(f"Saved to: {file_path}")

if __name__ == "__main__":
    generate_pinch_parameters()
