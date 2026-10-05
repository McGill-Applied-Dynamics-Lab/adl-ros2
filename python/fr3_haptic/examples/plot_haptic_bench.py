from pathlib import Path

from fr3_haptic.bench import HapticBench, HapticBenchConfig, plot_run, save_bench_run

base_dir = Path("data/bench_runs")
run_dir = base_dir / "bench_20260625_155246"

plot_run(run_dir)
