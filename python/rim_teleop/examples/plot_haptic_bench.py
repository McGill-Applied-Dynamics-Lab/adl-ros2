from pathlib import Path

from rim_teleop.bench import HapticBench, HapticBenchConfig, plot_run, save_bench_run

base_dir = Path("data/bench_runs")
run_dir = base_dir / "bench_20260623_162241"

plot_run(run_dir)
