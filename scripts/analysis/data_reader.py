import json
import os
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pandas as pd

from metrics import liftoff_t_ns


class DataReader:
    """Loads one CSV across every experiment at a time, so class Analysis
    controls when a file goes out of scope rather than holding all of them."""

    def __init__(self, logdir: str, workers: int | None = None,
                 time_reference: str = "mission", liftoff_threshold_m: float = 0.15):
        self.logdir = logdir
        self.experiment_ids = sorted(
            d
            for d in os.listdir(logdir)
            if os.path.isdir(os.path.join(logdir, d)) and d != "figures"
        )
        # workers=None resolves to one thread per experiment, capped at the
        # machine's core count; workers=1 disables pooling and every call
        # below falls back to the original sequential loop byte for byte.
        self.workers = workers if workers is not None else min(
            len(self.experiment_ids), os.cpu_count() or 1
        )
        self._io_pool = (
            ThreadPoolExecutor(max_workers=self.workers) if self.workers > 1 else None
        )
        self._epoch_t_ns = self._scan_epochs()
        # Lift off relative to the active epoch, per experiment, and with
        # time_reference "liftoff" the epoch itself, plans/analysis_module.md
        # section 19. An experiment without ground truth or without a lift
        # off keeps its mission epoch.
        self.time_reference = time_reference
        self.liftoff_t_ns: dict[str, int] = {}
        for exp, t_lift in self._scan_liftoffs(liftoff_threshold_m).items():
            if exp not in self._epoch_t_ns:
                continue
            if time_reference == "liftoff":
                self._epoch_t_ns[exp] = t_lift
            self.liftoff_t_ns[exp] = t_lift - self._epoch_t_ns[exp]

    def _scan_epoch_one(self, exp: str) -> tuple[str, int | None]:
        exp_dir = os.path.join(self.logdir, exp)
        mins = []
        for fname in os.listdir(exp_dir):
            if not fname.endswith(".csv"):
                continue
            try:
                col = pd.read_csv(
                    os.path.join(exp_dir, fname), usecols=["t_ns"]
                )["t_ns"]
            except (ValueError, pd.errors.EmptyDataError):
                continue
            if not col.empty:
                mins.append(int(col.min()))
        return exp, (min(mins) if mins else None)

    def _scan_epochs(self) -> dict[str, int]:
        """Every experiment is a separate simulation run with its own wall
        clock epoch, so one experiment's t_ns is not comparable to another's
        until each is offset by its own mission start. The reference has to
        be the same constant for every dataset belonging to one experiment,
        not each dataset's own first row, since the different logging sites
        (triangulation, the IMU frame record, the state trace, ...) start
        emitting rows at slightly different frames and an independent offset
        per file would misalign them against each other within one run. This
        scans every CSV in each experiment directory once, for the minimum
        t_ns across all of them, before any dataframe is handed out. One unit
        of work per experiment, dispatched through _io_pool when pooled,
        since pandas' C CSV parser releases the GIL during the parse itself."""
        pairs = (
            self._io_pool.map(self._scan_epoch_one, self.experiment_ids)
            if self._io_pool
            else (self._scan_epoch_one(exp) for exp in self.experiment_ids)
        )
        return {exp: m for exp, m in pairs if m is not None}

    def _scan_liftoffs(self, threshold_m: float) -> dict[str, int]:
        """Absolute t_ns of each experiment's lift off, from gt_eval.csv."""
        out = {}
        for exp in self.experiment_ids:
            path = os.path.join(self.logdir, exp, "gt_eval.csv")
            if not os.path.exists(path):
                continue
            try:
                df = pd.read_csv(path, usecols=["t_ns", "p_gt_2"]).dropna()
            except (ValueError, pd.errors.EmptyDataError):
                continue
            if df.empty:
                continue
            t_lift = liftoff_t_ns(df["t_ns"].to_numpy(), df["p_gt_2"].to_numpy(), threshold_m)
            if t_lift is not None:
                out[exp] = t_lift
        return out

    def csv_names(self) -> list[str]:
        """Every CSV base name present in at least one experiment, sorted."""
        names = set()
        for exp in self.experiment_ids:
            for f in os.listdir(os.path.join(self.logdir, exp)):
                if f.endswith(".csv"):
                    names.add(f[:-4])
        return sorted(names)

    def _load_one(self, name: str, exp: str) -> tuple[str, pd.DataFrame | None]:
        path = os.path.join(self.logdir, exp, f"{name}.csv")
        # The Logger writes a CSV's header with its first flush, so a table
        # with no flushed rows, or a run still being recorded, leaves a zero
        # byte file that pd.read_csv rejects with EmptyDataError.
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            return exp, None
        df = pd.read_csv(path)
        if df.empty:
            return exp, None
        if "t_ns" in df.columns and exp in self._epoch_t_ns:
            df["t_ns"] = df["t_ns"] - self._epoch_t_ns[exp]
        return exp, df

    def load(self, name: str) -> dict[str, pd.DataFrame]:
        """name is a CSV base name with no extension. An experiment missing
        the file, or with an empty file, is simply absent from the result.
        Any t_ns column is shifted by that experiment's own mission start
        (see _scan_epochs), so a plot overlaying several experiments compares
        them on mission relative time rather than on the wall clock moment
        each simulation happened to be launched. Dispatched one read per
        experiment through _io_pool when pooled, same rationale as
        _scan_epochs."""
        pairs = (
            self._io_pool.map(lambda exp: self._load_one(name, exp), self.experiment_ids)
            if self._io_pool
            else (self._load_one(name, exp) for exp in self.experiment_ids)
        )
        return {exp: df for exp, df in pairs if df is not None}

    def load_json(self, name: str) -> dict[str, dict]:
        out = {}
        for exp in self.experiment_ids:
            path = os.path.join(self.logdir, exp, f"{name}.json")
            if os.path.exists(path):
                with open(path) as fh:
                    out[exp] = json.load(fh)
        return out

    @staticmethod
    def vec(df: pd.DataFrame, prefix: str, n: int) -> np.ndarray:
        """Reassembles an (N, n) array from the prefix_0..prefix_{n-1} columns."""
        return df[[f"{prefix}_{i}" for i in range(n)]].to_numpy()

    @staticmethod
    def group_by(df: pd.DataFrame, keys: list[str]) -> dict[tuple, pd.DataFrame]:
        return {k: v for k, v in df.groupby(keys)}
