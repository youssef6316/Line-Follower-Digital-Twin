"""
logger.py — CSV Logging and Experiment Summary for Client 3
============================================================
Provides two loggers:

  EpisodeLogger   — streams one row per simulation step to a CSV file.
                    Used inside the VSI visualizer client main loop.

  ExperimentLogger — collects KPIResult objects across multiple episodes
                     (multiple gain sets / random spawns) and writes a
                     summary CSV + console table at the end of each
                     experiment (E1–E4).

Output directory structure (created automatically):
  results/
  ├── e1_gain_sweep/
  │   ├── run_G1_sluggish_spawn0.csv
  │   ├── run_G3_baseline_spawn0.csv
  │   └── summary_e1.csv
  ├── e2_curved_path/
  │   └── ...
  ├── e3_noise_rejection/
  │   └── ...
  └── e4_pd_vs_pid/
      └── ...
"""

import csv
import os
import time
from dataclasses import dataclass
from typing import Optional
from client3_visualizer.kpi import KPIResult


# ---------------------------------------------------------------------------
# Episode (step-level) logger
# ---------------------------------------------------------------------------

class EpisodeLogger:
    """
    Writes one CSV row per simulation step for a single episode.

    Columns:
      t, x, y, theta, x_true, y_true, theta_true,
      v, omega, e_lat, e_head, path_progress
    """

    COLUMNS = [
        "t", "x", "y", "theta",
        "x_true", "y_true", "theta_true",
        "v", "omega", "e_lat", "e_head", "path_progress",
    ]

    def __init__(self, filepath: str):
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        self._file   = open(filepath, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self.COLUMNS,
                                      extrasaction="ignore")
        self._writer.writeheader()
        self._filepath = filepath

    def log(self, **kwargs) -> None:
        """
        Write one row.  Pass keyword args matching COLUMNS; extras ignored.
        Round floats to 6 decimal places for compact files.
        """
        row = {k: round(v, 6) if isinstance(v, float) else v
               for k, v in kwargs.items()}
        self._writer.writerow(row)

    def close(self) -> None:
        self._file.flush()
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


# ---------------------------------------------------------------------------
# Experiment (episode-level) summary logger
# ---------------------------------------------------------------------------

class ExperimentLogger:
    """
    Collects KPIResult objects and writes a summary CSV + console table.

    Usage
    -----
    exp = ExperimentLogger("e1_gain_sweep", results_dir="results")
    exp.add(kpi_result)
    ...
    exp.finalize()   # writes summary CSV and prints table
    """

    def __init__(self, experiment_name: str, results_dir: str = "results"):
        self.name      = experiment_name
        self.outdir    = os.path.join(results_dir, experiment_name)
        os.makedirs(self.outdir, exist_ok=True)
        self._records: list[KPIResult] = []

    def add(self, kpi: KPIResult) -> None:
        self._records.append(kpi)

    def episode_csv_path(self, gains_name: str, spawn_idx: int) -> str:
        """Return filepath for a per-episode step log."""
        fname = f"run_{gains_name}_spawn{spawn_idx}.csv"
        return os.path.join(self.outdir, fname)

    def finalize(self) -> str:
        """
        Write summary CSV and print a formatted console table.
        Returns the path to the summary CSV.
        """
        if not self._records:
            print(f"[ExperimentLogger] No records to write for '{self.name}'.")
            return ""

        summary_path = os.path.join(self.outdir, f"summary_{self.name}.csv")
        fieldnames   = list(self._records[0].as_dict().keys())

        with open(summary_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for rec in self._records:
                writer.writerow(rec.as_dict())

        self._print_table()
        print(f"\n  Summary saved → {summary_path}")
        return summary_path

    def _print_table(self) -> None:
        """Pretty-print KPI table to stdout."""
        header = (
            f"\n{'─'*78}\n"
            f"  {'Gains':<22} {'Overshoot(m)':>12} "
            f"{'Settle(s)':>10} {'SS Err(m)':>10} {'Settled':>8}\n"
            f"{'─'*78}"
        )
        print(header)
        for r in self._records:
            print(
                f"  {r.gains_name:<22} "
                f"{r.overshoot_m:>12.4f} "
                f"{r.settling_time_s:>10.3f} "
                f"{r.steady_state_error_m:>10.4f} "
                f"{'YES' if r.settled else 'NO':>8}"
            )
        print(f"{'─'*78}")
