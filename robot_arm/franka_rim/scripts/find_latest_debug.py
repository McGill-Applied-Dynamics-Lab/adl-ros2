#!/usr/bin/env python3
"""
Find and plot the latest DelayRIM debug file.
"""

from pathlib import Path
import subprocess
import sys


def main():
    debug_dir = Path("/tmp/delay_rim_debug")

    if not debug_dir.exists():
        print("No debug directory found: /tmp/delay_rim_debug")
        sys.exit(1)

    csv_files = list(debug_dir.glob("delay_rim_debug_*.csv"))
    if not csv_files:
        print("No debug files found in /tmp/delay_rim_debug/")
        sys.exit(1)

    latest_file = max(csv_files, key=lambda f: f.stat().st_mtime)
    print(f"Latest debug file: {latest_file}")

    # Run plotting script
    plot_script = Path(__file__).parent / "plot_delay_rim_debug.py"
    subprocess.run([sys.executable, str(plot_script), str(latest_file)])


if __name__ == "__main__":
    main()
