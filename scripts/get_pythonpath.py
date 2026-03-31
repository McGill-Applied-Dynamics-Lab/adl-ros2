#!/usr/bin/env python3
"""
Generates the PYTHONPATH for debugging ROS2 packages.
Run this after building and copy the output to your launch.json
"""
import os
from pathlib import Path

workspace = Path(__file__).parent.parent
install_dir = workspace / "install_humble"
src_dir = workspace / "src"

pythonpath_parts = [str(src_dir)]

# Find all site-packages directories in install_humble
for package_dir in sorted(install_dir.iterdir()):
    if package_dir.is_dir():
        site_packages = package_dir / "lib/python3.12/site-packages"
        if site_packages.exists():
            pythonpath_parts.append(str(site_packages))

pythonpath = ":".join(pythonpath_parts)
print(pythonpath)
