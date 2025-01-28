import warnings

# Suppress the specific UserWarning about NumPy version
warnings.filterwarnings("ignore", category=UserWarning, message="A NumPy version >=")
