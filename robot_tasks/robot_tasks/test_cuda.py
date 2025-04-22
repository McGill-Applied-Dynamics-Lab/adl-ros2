import torch
import gymnasium

print(gymnasium.__version__)


print(torch.__version__)

# If you installed a CUDA version, check if it's available:
print(f"PyTorch CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA version used by PyTorch: {torch.version.cuda}")
    print(f"Device name: {torch.cuda.get_device_name(0)}")

...
