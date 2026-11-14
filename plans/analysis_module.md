# SLAM Analysis and Visualisation Dashboard

## 1. Motivation

`plans/vio_divergence_fix.md` establishes that the accelerometer bias runaway diagnosed there was found by reading interleaved `std::cout` prose with `scripts/vio_log_extract.py`, a script whose own docstring records that it carries four separate defences against corrupted records and still rejected 699 of them on one capture, one of which inflated a measured latency from 18 ms to 52 seconds and would have redirected the whole investigation had it not been caught. `plans/SLAM_logging.md` replaces that console prose with the `Logger`/`AggregateStats` CSV system audited in `context/slam_analysis.md`, so every quantity the divergence investigation needed, and everything `context/slam_analysis.md`'s literature survey adds on top of it, now lands in a structured, directly loadable table with no reconstruction step. `scripts/vio_log_plot.py` still only draws one run's own PNGs or overlays every run on the same axes with no way to select a subset, and produces no scalar comparison table at all, so judging whether a configuration change helped still means reading numbers off overlapping line plots by eye. The goal of this plan is the tool that closes that gap, an offline dashboard that reads any number of runs of the same dataset, computes every metric `context/slam_analysis.md` catalogues, and renders one static, interactive, remotely viewable page in which a run can be selected, deselected, and compared against the others on every plot and in one sortable, filterable metrics table.

`scripts/vio_log_extract.py` reconstructs exactly the structured record the `Logger` now writes directly, so it has no further purpose once a run passes through the new CSV path, and is deprecated by this plan along with `scripts/vio_log_plot.py`, whose static, per run PNG output is superseded by the interactive, multi run dashboard specified below. Section 13 records the deprecation itself and the one prior planning decision it corrects.

## 2. Architecture

### 2.1 Data flow

```
--logdir slam/
  2026-09-25-19-51-11/   <- one basalt run, config.json + calibration.json + ~40 CSVs
  2026-09-26-01-15-16/
  2026-09-26-02-25-46/
        |
        v
  class DataReader          one CSV name in, dict[experiment_id -> DataFrame] out,
  (data_reader.py)           one CSV loaded across every experiment at a time
        |
        v
  class Analysis             one _analyse_<tab>() method per subsystem tab, each
  (analysis.py)               pulling only the CSVs that tab needs from DataReader
        |              |
        v              v
  class TrajectoryAnalysis   metrics.py / visualisation.py
  (trajectory_analysis.py)   stateless functions, data + hyperparameters in,
        |                     a scalar/DataFrame or a plotly/bokeh Figure out
        v
  class Alignment
  (alignment.py)
        |
        v
  slam/metrics.csv  +  slam/figures/<tab>/<name>.html  +  a live page, panel serve
```

**Correction, 2026-10-20.** This last line originally read `slam/dashboard.html + slam/metrics.csv + slam/figures/<tab>/<name>.html`, a third static file alongside the other two. `dashboard.py` no longer writes a `dashboard.html`, `plans/dashboard_live_migration.md`, the dashboard itself is a live `panel serve` page, not a file `class Analysis` or `dashboard.py` write to `slam/`; `metrics.csv` and `figures/` are unaffected.

`class Analysis` is the only class that reads `config.json`, the only class that writes to disk, and the only class that calls both `class TrajectoryAnalysis` and the bare `metrics.py`/`visualisation.py` functions, which matches requirement 11's hierarchy, `class Analysis` on top, `class TrajectoryAnalysis` beneath it, `class Alignment` beneath that. `class DataReader` is stateless with respect to `class Analysis`, it is handed a CSV name and returns fresh DataFrames, so `class Analysis` decides when a file is loaded and when it goes out of scope, keeping the memory profile requirement 5 asks for without either class needing to track what has already been read.

### 2.2 File layout under `scripts/analysis/`

| File | Contents |
|---|---|
| `dashboard.py` | `--logdir`/`--config` parsed from `sys.argv`, constructs `DataReader` and `Analysis`, calls `Analysis.run()`, serves the tabs and metrics table as a live `panel serve` app rather than saving `dashboard.html` (corrected 2026-10-20, `plans/dashboard_live_migration.md` §4.3) |
| `data_reader.py` | `class DataReader` |
| `alignment.py` | `class Alignment`, the three Umeyama modes from `context/slam_analysis.md` section 2.1 |
| `trajectory_analysis.py` | `class TrajectoryAnalysis`, ATE, and the CuPy batched RPE from `context/slam_analysis.md` section 2.2 |
| `metrics.py` | stateless scalar and DataFrame producing functions, one per non trivial derived quantity |
| `visualisation.py` | stateless Figure producing functions, one per plot shape, reused across many quantities; plotly for every shape except `time_series`/`multi_axis_time_series`/`categorical_timeline`, native Bokeh for those three since 2026-10-20 so their `x_range` can be shared across tabs, `plans/dashboard_live_migration.md` §4.1 |
| `config.json` | every plot and metric's `enabled` flag and hyperparameters, section 6 below |
| `README.md` | generated in Phase 5, documents every `config.json` key and every plot, written once the code exists so it can be checked against the running dashboard rather than drafted blind |

This is exactly the class list requirement 9 and requirement 11 name, one file per class plus the two stateless modules requirement 10 names by their own filenames, no further split.

### 2.3 Library choice for the dashboard shell

Every plot is a `plotly.graph_objects.Figure` regardless of which assembly library wraps it, so this choice touches only `dashboard.py`, not `metrics.py`, `visualisation.py` or `class Analysis`. Four candidates were compared against requirement 16's own criteria, ease of use, install size, functionality and speed, applied here to the assembly layer rather than to the GPU question section 6's own comparison already settled. Plotly's own legend click toggling is confirmed directly from its documentation, "users may show or hide traces by clicking or double-clicking on their associated legend item" `[26]`, and holds regardless of which of the four wraps the figure, so it is not itself a discriminating factor.

| Criterion | HoloViz Panel | Bokeh alone | Hand rolled, Plotly + itables | Datapane |
|---|---|---|---|---|
| Tabs | `pn.Tabs`, a pure display container, tab switch is client side with no Python involved `[27]` | `bokeh.models.Tabs`/`TabPanel`, the same primitive Panel itself is built on `[29]` | A `<div>` per tab plus a few lines of vanilla JavaScript, no library | N/A |
| Filterable metrics table | `pn.widgets.Tabulator(header_filters=True)`, documented client side filtering, "the Tabulator widget also offers a client side filtering API... exposed through header_filters" `[27]`, no Python round trip | `CustomJSFilter` + `CDSView`, a documented but hand written JavaScript callback pattern maintaining two data sources `[29][30]` | `itables.to_html_datatable`, a fully offline bundled DataTables.js template, verified from source at `src/itables/javascript.py` `[31]`, plus a small per column JavaScript snippet for the category filter | N/A |
| Static, no server export | `panel.save('dashboard.html', resources=INLINE)`, confirmed static `[28]` | `bokeh.embed.file_html`, confirmed static `[29]` | Always static, it is plain HTML | Confirmed static in principle |
| Install footprint | 12 hard dependencies including Bokeh, Param and Tornado `[32]`, the heaviest of the four | 13 hard dependencies `[32]` | 2 hard dependencies, Plotly's own `narwhals`/`packaging` `[32]`, `itables` adds 0 `[31][32]` | N/A |
| Maintenance | Active | Active | Active, since it is this stack's own Plotly plus a small template | Archived in substance, last published release pins `pandas<2.0` and `plotly<6.0` against current versions `[33]` |

Datapane is ruled out outright on the last row. Bokeh alone would work but its table filtering is the more verbose, hand maintained `CustomJSFilter` pattern for the same requirement Panel gives as one constructor argument. The verdict here was HoloViz Panel, `pn.Tabs` holding one `pn.pane.Plotly(fig)` per plot per subsystem tab and `pn.widgets.Tabulator(metrics_df, header_filters=True)` for the metrics table, saved with `dashboard.save("dashboard.html", resources=INLINE)`, kept as the record of why that choice was made at the time, `plotly.graph_objects.Figure` is still what `histogram`/`stacked_bar`/`heatmap`/`line_3d`/`trajectory_3d_overlay`/`diff_table`/`box_violin`/`multi_axis_histogram`/`histogram_prebinned`/`line`/`line_with_band` produce and `pn.pane.Plotly` still renders them. It no longer describes `time_series`/`multi_axis_time_series`/`categorical_timeline` or how the page is served. **Correction, 2026-10-20.** Those three shapes, the only ones that ever need figures on different tabs to zoom together, are now native Bokeh figures sharing one `bokeh.models.Range1d`, Bokeh's own linked-panning mechanism, and `dashboard.py` is now a live `panel serve` app rather than a `.save()`'d static file, because genuine cross-tab linked zoom together with genuine on-demand tab loading turned out to need a live server either way, `pn.Tabs(dynamic=True)` only truly defers a tab's render while one is attached. `plans/dashboard_live_migration.md` is the design record for this, the full rationale, the Bokeh replacements for those three functions, and the `dashboard.py`/`djinn`/Docker changes that follow from serving rather than saving.

Panel's own dependency weight, twelve packages against Plotly's two, was recorded here as the cost of the original verdict rather than treated as free, and the documented static fallback below is kept for its own sake, a constrained or air gapped install target with no server to run at all would still want it. Should that weight prove a problem in whichever environment eventually runs `dashboard.py`, the fallback is the hand rolled column of the table above, `plotly.io.to_html(fig, full_html=False, include_plotlyjs=...)` fragments behind a small Bootstrap tab strip, with `itables.to_html_datatable(df, connected=False)` supplying the metrics table from its own fully offline bundled template and a short per column JavaScript snippet added for the category filter Tabulator would otherwise give for free. This fallback predates, and is independent of, the live-server design `plans/dashboard_live_migration.md` now uses instead of the plain `.save()` path described above.

The live dashboard page itself, since 2026-10-20 a `panel serve` app rather than a saved file (`plans/dashboard_live_migration.md`), is reached over a live connection and so is not and does not need to be offline; `resources=INLINE` is no longer part of how it is served. The standalone files `figures/` needs are still written independently by `class Analysis` itself, `fig.write_html(path, include_plotlyjs=True)` for Plotly figures, `bokeh.io.save(path, resources=INLINE)` for the Bokeh ones since the same date, embedding each library's own JavaScript inline in every file so a single figure copied elsewhere for individual analysis, requirement 3's own third output, still opens with no internet reachable, exactly the offline requirement `context/slam_analysis.md` section 8 already reads into `rerun.io` and Foxglove's own saved layout philosophy.

## 3. Output contract

Per requirement 3, running `dashboard.py --logdir slam/` originally wrote a `slam/dashboard.html` alongside `metrics.csv` and `figures/`. **Correction, 2026-10-20.** `dashboard.py` is now a `panel serve` app rather than a script that saves a file, `plans/dashboard_live_migration.md`, so no `dashboard.html` is written at all any more, the dashboard itself is a live page at whatever port `panel serve` binds, not a file under `slam/`. `metrics.csv` and `figures/` are unaffected and still written exactly as below, that part of this contract still holds.

```
slam/
  metrics.csv
  figures/
    trajectory/*.html
    optical_flow_detection/*.html
    optical_flow_tracking/*.html
    vio_triangulation/*.html
    vio_optimisation/*.html
    mapping_detection/*.html
    mapping_matching/*.html
    mapping_triangulation/*.html
    mapping_optimisation/*.html
    timing/*.html
    cross_run/*.html
```

Every file under `figures/` is written with `include_plotlyjs=True` (the Bokeh-backed figures among them, `plans/dashboard_live_migration.md` §4.2, written with `bokeh.io.save(..., resources=INLINE)` instead), embedding its own JavaScript inline either way, so any one of them opens standalone with no internet reachable and no live `panel serve` process needed, independent of the live dashboard page.

`metrics.csv` has one row per metric and one column per experiment, per the explicit requirement that rows be metrics and columns be experiments, since a run typically computes many more scalar metrics than the handful of experiments being compared. `category`, `name` and `type` are its first three columns, `category` naming the tab the metric belongs to, `name` a human phrase such as `"ATE (m)"`, `type` the statistic kind such as `"rmse"`/`"mean"`/`"std"`/`"percentile"`/`"rate"`/`"value"`, the two split so two rows can share a name and differ only by type, so the same file that feeds the dashboard's table can also be filtered by any external tool with no `panel serve` process running. `metrics.pivot_metrics`, section 8, is the one function that produces this shape, called identically by `Analysis.run()` for the CSV and by `dashboard.py` for the Tabulator table, so the two can never present two different layouts of the same data. The live dashboard's own first tab, `scalars`, is this same table rendered as a filterable, sortable `pn.widgets.Tabulator`, section 10, its `category` column filtered by an explicit dropdown of the categories present rather than free text, every column sized to its own longest cell so no name or category string is ever wrapped or truncated, so no scalar quantity, drift proxies and initialisation sanity readings included, is ever rendered as a standalone tile shaped figure, every scalar lives in exactly one table a reader opens once rather than scattered one tile at a time across whichever subsystem tab happened to compute it.

## 4. Tab taxonomy

Requirement 15 names the filter classes the metrics table must support directly, trajectory, optical flow detection, optical flow tracking, vio triangulation, vio optimisation, mapping triangulation, mapping matching, mapping detection, and this plan uses that list itself as the tab taxonomy rather than inventing a second one, since a plot and its own scalar summary should sit behind the same filter label. `dashboard.html` itself carries a twelfth, leading tab, `scalars`, section 3, built directly in `dashboard.py` from `self.metrics_rows` rather than by an `_analyse_scalars` method of its own, since every category it aggregates is already produced by the eleven tabs below. The full eleven `_analyse_*` tabs, with every quantity `context/slam_analysis.md` catalogues placed in exactly one of them, are given in section 7's routing table. One placement departs from `context/slam_analysis.md`'s own section boundaries and is recorded here so the departure reads as a decision rather than an inconsistency. `vio_triang.csv`'s parallax angle and baseline distribution are tabled under section 3 of `context/slam_analysis.md` because they are a front end diagnostic in spirit, but they are moved into the `vio_triangulation` tab here because they share a file and a rejection funnel with the rest of that tab's content and the user's own filter list names `vio triangulation` as one category rather than splitting it across two. Every timing quantity this dashboard logs, front end, VIO, VIO solver, VIO marginalisation, mapper cycle, mapper detection and mapper matching alike, lives in the `timing` tab and nowhere else, `mapper_detect.csv`'s own detection latency and `map_match_summary.csv`'s own db query and matching latency included, so a reader comparing timing across the whole pipeline opens exactly one tab rather than piecing it together from `mapping_detection` and `mapping_matching` as well. `mapping_detection` and `mapping_matching` keep only the non timing quantities their own subsystem produces, detector yield, track length, BoW retrieval and match funnels among them.

## 5. class DataReader

Every experiment is a separate simulation run with its own wall clock epoch, so one run's `t_ns` is not directly comparable to another's, and a dashboard that overlays several runs on one time axis needs them expressed on a shared, mission relative clock instead. The reference point has to be one constant per experiment, not one constant per file, because the different logging sites, triangulation, the IMU frame record, the state trace and the rest, start emitting rows at slightly different frames within the same run, and an independent per file offset would misalign them against each other even within one experiment. `DataReader` therefore scans every CSV in an experiment's directory once at construction, before any dataframe is handed out, and takes the minimum `t_ns` across all of them as that experiment's own mission start. `load()` then subtracts this one constant from the `t_ns` column of whatever file it returns, so every quantity from every file in one experiment sits on the same zero point, and every experiment's zero point is its own mission start rather than the moment its simulation happened to be launched.

```python
# data_reader.py
import json
import os

import numpy as np
import pandas as pd


class DataReader:
    """Loads one CSV across every experiment at a time, so class Analysis
    controls when a file goes out of scope rather than holding all of them."""

    def __init__(self, logdir: str):
        self.logdir = logdir
        self.experiment_ids = sorted(
            d
            for d in os.listdir(logdir)
            if os.path.isdir(os.path.join(logdir, d)) and d != "figures"
        )
        self._epoch_t_ns = self._scan_epochs()

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
        t_ns across all of them, before any dataframe is handed out."""
        epochs = {}
        for exp in self.experiment_ids:
            exp_dir = os.path.join(self.logdir, exp)
            mins = []
            for fname in os.listdir(exp_dir):
                if not fname.endswith(".csv"):
                    continue
                try:
                    col = pd.read_csv(os.path.join(exp_dir, fname), usecols=["t_ns"])["t_ns"]
                except (ValueError, pd.errors.EmptyDataError):
                    continue
                if not col.empty:
                    mins.append(int(col.min()))
            if mins:
                epochs[exp] = min(mins)
        return epochs

    def load(self, name: str) -> dict[str, pd.DataFrame]:
        """name is a CSV base name with no extension. An experiment missing
        the file, or with an empty file, is simply absent from the result.
        Any t_ns column is shifted by that experiment's own mission start
        (see _scan_epochs), so a plot overlaying several experiments compares
        them on mission relative time rather than on the wall clock moment
        each simulation happened to be launched."""
        out = {}
        for exp in self.experiment_ids:
            path = os.path.join(self.logdir, exp, f"{name}.csv")
            if not os.path.exists(path):
                continue
            df = pd.read_csv(path)
            if df.empty:
                continue
            if "t_ns" in df.columns and exp in self._epoch_t_ns:
                df["t_ns"] = df["t_ns"] - self._epoch_t_ns[exp]
            out[exp] = df
        return out

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
```

Every bespoke reshape a specific file needs, joining `map_bow_candidate.csv` against `gt_eval.csv` by nearest timestamp for the BoW precision recall label for instance, is a small function in `metrics.py` that calls `DataReader.load` and `DataReader.vec`/`group_by` directly rather than a further method on `class DataReader` itself, since `context/slam_analysis.md`'s own routing table in section 7 below shows that all but a handful of quantities need exactly one file read in exactly one of the two generic shapes above, and the handful that need a genuine join are one function each in `metrics.py`, not a new abstraction.

## 6. class Alignment and class TrajectoryAnalysis

`alignment.py` is `context/slam_analysis.md` section 2.1's `align_umeyama` function and its three wrappers, moved here verbatim as the file that actually ships, with the context file kept as the derivation and the literature citations, `[2]`, `[3]`, `[10]`, `[11]`, behind it.

```python
# alignment.py
import numpy as np

def _get_best_yaw(C: np.ndarray) -> float:
    """Closed form theta maximising trace(Rz(theta) . C), rpg_trajectory_evaluation's
    align_trajectory.py, [3]. C is the 3x3 cross covariance data^T . model,
    context/slam_analysis.md section 2.1, quoted here verbatim."""
    a = C[0, 1] - C[1, 0]
    b = C[0, 0] + C[1, 1]
    return np.pi / 2 - np.arctan2(b, a)

def _rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

def _align_umeyama(model: np.ndarray, data: np.ndarray, known_scale: bool = False,
                    yaw_only: bool = False):
    """model = s * R * data + t. model and data are Nx3 point sets, rows are
    points, context/slam_analysis.md section 2.1, adapted from
    rpg_trajectory_evaluation's own align_trajectory.py/align_utils.py, [3],
    itself S. Umeyama, "Least-Squares Estimation of Transformation Parameters
    Between Two Point Patterns," IEEE PAMI 13(4), 1991. A general purpose SVD
    of the cross covariance is always computed for the scale term, and is
    separately overridden by the closed form yaw only rotation when yaw_only
    is set, so every one of the three dispatch modes below shares one code
    path rather than three. Numerically verified against synthetic
    trajectories to ~1e-15 error for all three modes."""
    mean_model = model.mean(0)
    mean_data = data.mean(0)
    model_zerocentered = model - mean_model
    data_zerocentered = data - mean_data
    n = np.shape(model)[0]

    cov = 1.0 / n * np.dot(model_zerocentered.T, data_zerocentered)
    U_svd, D_svd, V_svd = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U_svd) * np.linalg.det(V_svd) < 0:
        S[2, 2] = -1

    if yaw_only:
        rot_C = np.dot(data_zerocentered.T, model_zerocentered)
        theta = _get_best_yaw(rot_C)
        R = _rot_z(theta)
    else:
        R = np.dot(U_svd, np.dot(S, V_svd))

    if known_scale:
        s = 1.0
    else:
        sigma2 = 1.0 / n * np.multiply(data_zerocentered, data_zerocentered).sum()
        s = 1.0 / sigma2 * np.trace(np.dot(np.diag(D_svd), S))

    t = mean_model - s * np.dot(R, mean_data)
    return s, R, t

class Alignment:
    """Three Umeyama alignment modes, context/slam_analysis.md section 2.1.
    sim3 for monocular VO, scale unobservable. se3 for stereo VO, scale metric
    but no gravity reference. posyaw for gravity aligned VIO, roll and pitch
    already observable from the accelerometer and must not be re-absorbed.
    Every mode takes gt as model and est as data, so applying the returned
    (s, R, t) to est recovers gt."""

    @staticmethod
    def sim3(gt_xyz: np.ndarray, est_xyz: np.ndarray):
        """Monocular VO, scale unobservable, full 7 degree of freedom similarity."""
        return _align_umeyama(gt_xyz, est_xyz, known_scale=False, yaw_only=False)

    @staticmethod
    def se3(gt_xyz: np.ndarray, est_xyz: np.ndarray):
        """Stereo VO, scale metric from the baseline, no gravity reference, all
        three rotation degrees of freedom still need aligning."""
        _, R, t = _align_umeyama(gt_xyz, est_xyz, known_scale=True, yaw_only=False)
        return 1.0, R, t

    @staticmethod
    def posyaw(gt_xyz: np.ndarray, est_xyz: np.ndarray):
        """Gravity aligned VIO, roll and pitch observable via the accelerometer
        and must not be absorbed into alignment, only translation and yaw are
        genuinely unobservable, see the Hesch/OpenVINS nullspace in section 4."""
        _, R, t = _align_umeyama(gt_xyz, est_xyz, known_scale=True, yaw_only=True)
        return 1.0, R, t
```

Every mode now returns the same three tuple `(s, R, t)`, `se3`/`posyaw` fixing `s=1.0` explicitly rather than dropping it, so `class TrajectoryAnalysis` below never needs to branch on how many values a mode returned.

`class TrajectoryAnalysis` wraps `class Alignment` with the pose construction, the ATE definition `context/slam_analysis.md` section 2 states in prose but does not yet give as code, and the RPE sweep the same document's section 2.2 already gives as code in full. Per the feedback that alignment must be computed once per experiment and reused, not recomputed inside `ate()` and again inside `rpe()`, alignment is a two step protocol, `set_alignment()` computes and stores `self.R`/`self.t`/`self.s` once, every later `ate()`/`rpe()` call on the same instance reuses them, and `reset_alignment()` clears them so a stale alignment from one experiment can never leak into the next one's numbers by accident. `apply()` itself is also called exactly once per experiment, by `class Analysis`, not by `ate()`/`rpe()` internally, so a second round of feedback correcting the first draft of this section, `ate()` and `rpe()` now accept the already aligned trajectory the caller's own single `apply()` call produced rather than re-deriving it, which is the difference between paying for one rotation and translation of the whole trajectory per experiment and paying for it up to three times.

```python
# trajectory_analysis.py
import numpy as np
from scipy.spatial.transform import Rotation
from alignment import Alignment
from metrics import relative_pose_error_sweep

def poses_to_se3(p_xyz: np.ndarray, q_xyzw: np.ndarray) -> np.ndarray:
    """(N, 3) and (N, 4) Eigen coeffs() ordered quaternions, confirmed at
    src/controller.cpp:255-256, to (N, 4, 4) SE(3). scipy's own from_quat
    expects the same x, y, z, w order, so no reordering is needed."""
    n = p_xyz.shape[0]
    T = np.zeros((n, 4, 4))
    T[:, :3, :3] = Rotation.from_quat(q_xyzw).as_matrix()
    T[:, :3, 3] = p_xyz
    T[:, 3, 3] = 1.0
    return T

class TrajectoryAnalysis:
    ALIGN = {"sim3": Alignment.sim3, "se3": Alignment.se3, "posyaw": Alignment.posyaw}

    def __init__(self, mode: str):
        if mode not in self.ALIGN:
            raise ValueError(f"unknown alignment mode {mode!r}, expected one of {list(self.ALIGN)}")
        self.mode = mode
        self.R = None
        self.t = None
        self.s = 1.0

    def set_alignment(self, gt_xyz: np.ndarray, gt_qxyzw: np.ndarray,
                       est_xyz: np.ndarray, est_qxyzw: np.ndarray) -> None:
        """Computes this experiment's own alignment once and stores it on
        self.R/self.t/self.s. gt_qxyzw/est_qxyzw are accepted for a call
        signature uniform with ate()/rpe() even though alignment.py's own
        three modes are position only. Call once per experiment before ate()
        or rpe(), both reuse the stored alignment rather than resolving the
        same SVD twice."""
        self.s, self.R, self.t = self.ALIGN[self.mode](gt_xyz, est_xyz)

    def reset_alignment(self) -> None:
        """Clears the stored alignment between experiments so a caller cannot
        accidentally read one experiment's R/t/s against another's poses."""
        self.R, self.t, self.s = None, None, 1.0

    def apply(self, est_xyz: np.ndarray, est_qxyzw: np.ndarray):
        """Applies the stored alignment to one trajectory. Called exactly once
        per experiment by class Analysis, which passes the result on to
        ate()/rpe() below and reuses it again for the 3D and 2D trajectory
        plots, so the rotation and translation of the whole trajectory this
        method performs is paid for once per experiment, not once per
        consumer."""
        if self.R is None:
            raise RuntimeError("TrajectoryAnalysis.set_alignment() must be called before ate()/rpe()/apply()")
        est_xyz_aligned = (self.s * (self.R @ est_xyz.T)).T + self.t
        est_q_aligned = (Rotation.from_matrix(self.R) * Rotation.from_quat(est_qxyzw)).as_quat()
        return est_xyz_aligned, est_q_aligned

    def ate(self, gt_xyz: np.ndarray, est_xyz_aligned: np.ndarray) -> dict:
        """F_i = Q_i^-1 S P_i, context/slam_analysis.md section 2, S the
        alignment set_alignment() already computed and apply() already
        applied. Takes the ALIGNED estimate directly, est_xyz_aligned being
        apply()'s own first return value, rather than calling apply() again,
        per the feedback that the same alignment must not be paid for twice
        when a caller wants both ate() and rpe() for one experiment. Reports
        the same statistics evo_ape reports, [2], plus the aligned per frame
        residual array a colour mapped trajectory plot needs."""
        residual = np.linalg.norm(est_xyz_aligned - gt_xyz, axis=1)
        return {
            "rmse": float(np.sqrt(np.mean(residual ** 2))),
            "mean": float(residual.mean()), "median": float(np.median(residual)),
            "std": float(residual.std()), "min": float(residual.min()),
            "max": float(residual.max()), "sse": float(np.sum(residual ** 2)),
            "residual": residual,
        }

    def rpe(self, gt_xyz: np.ndarray, gt_qxyzw: np.ndarray, est_xyz_aligned: np.ndarray,
            est_q_aligned: np.ndarray, deltas: list[int], xp=None):
        """Sweeps every delta over the ALIGNED estimate, est_xyz_aligned and
        est_q_aligned being apply()'s own two return values, context/slam_analysis.md
        section 2.2's relative_pose_error_sweep, CuPy when available. Takes
        the aligned pose arrays directly rather than calling apply() again,
        same reason as ate() above."""
        T_est = poses_to_se3(est_xyz_aligned, est_q_aligned)
        T_gt = poses_to_se3(gt_xyz, gt_qxyzw)
        return relative_pose_error_sweep(T_est, T_gt, deltas, xp=xp)
```

## 7. Quantity routing table

Every row below is one entry from `context/slam_analysis.md` sections 2 through 9, mapped onto a tab, a `config.json` key, the file or files it reads, and the function that produces it. `Reader` names the `DataReader` call, bare when it is `load(name)` alone, `vec(df, prefix, n)` or `group_by(df, keys)` when a reshape is needed. `Vis fn` and `Metric fn` name the `visualisation.py`/`metrics.py` function, reused across rows wherever the plot shape repeats, so the two files hold on the order of fifteen functions between them for the forty five rows below.

### 7.1 Trajectory

| Quantity | Config key | Reader | Metric fn | Vis fn |
|---|---|---|---|---|
| Aligned ATE RMSE | `ate` | `load("gt_eval")`, drop blank rows | `TrajectoryAnalysis.ate` | scalar, into `metrics.csv` |
| Error coloured trajectory | `trajectory_3d.color_by_error` | same, `vec(df,"p_w_i",3)`/`vec(df,"p_gt",3)` | `TrajectoryAnalysis.ate`'s `residual` | `visualisation.line_3d(xyz, colour=residual)` |
| 3D trajectory overlay | `trajectory_3d` | same | `TrajectoryAnalysis._aligned` | `visualisation.line_3d(xyz_per_run)` |
| Ground truth and estimated trajectory overlay | `trajectory_3d_gt_overlay` | same | `metrics.combine_ground_truth(t_ns_per_run, gt_xyz_per_run)` | `visualisation.trajectory_3d_overlay(gt_xyz, xyz_per_run)` |
| 2D trajectory vs time, ground truth overlaid | `trajectory_2d` | same, plus `metrics.combine_ground_truth`'s own combined series | none, plotted directly | `visualisation.time_series(t_ns, xyz plus ground truth, axes_overlayed)` |
| Aligned ATE residual vs time | `trajectory_error` | same | aligned residual norm | `visualisation.time_series(t_ns, err)` |
| Aligned ATE residual vs time, by axis | `trajectory_error_xyz` | same | `est_xyz_aligned - gt_xyz` | `visualisation.multi_axis_time_series`, one subplot per axis |
| RPE RMSE vs length | `rpe_rmse` | same | `TrajectoryAnalysis.rpe` | `visualisation.line_with_band(curve)` |
| RPE per start time | `rpe_length` | same | `TrajectoryAnalysis.rpe`'s `per_delta_series` | `visualisation.time_series(t_ns[:-delta], series)` |
| Yaw offset at seeded reinit | `yaw_offset` | `load("vio_init")` | none | scalar, into `metrics.csv` |
| Closure, scale, rotation and translation drift | `drift_proxies` | `load("gt_eval")` | `metrics.drift_proxies(...)` | scalar, into `metrics.csv` |
| Tracking success | `tracking_success` | none | not implemented, open item | N/A |

### 7.2 Optical flow detection

| Quantity | Config key | Reader | Metric fn | Vis fn |
|---|---|---|---|---|
| Detection count per frame | `detection_count` | `load("of_frame")` | none | `visualisation.time_series` |
| Occupancy grid over time | `occupancy_grid` | none | not logged, open item | N/A |

### 7.3 Optical flow tracking

| Quantity | Config key | Reader | Metric fn | Vis fn |
|---|---|---|---|---|
| Track length distribution | `track_length_distribution` | `load("of_track_death")` | none | `visualisation.histogram(age_frames)` |
| Track survival curve | `track_survival_curve` | same | `metrics.survival_curve(age_frames)` | `visualisation.time_series` |
| Death reason breakdown | `death_reason_breakdown` | same | `df.groupby("reason").size()` | `visualisation.stacked_bar` |
| Reprojection error distribution | `reprojection_error_distribution` | `load("vio_reproj_pre_opt")` | none | `visualisation.histogram`/`time_series`, `type` list fans out to both |
| Inlier to outlier ratio | `inlier_outlier_ratio` | `load("of_frame")` | `tracked/(tracked+epi_rej)` | `visualisation.time_series` |
| Optical flow endpoint error | `optical_flow_endpoint_error` | `load("of_frame")` | none | `visualisation.time_series`+`histogram` |

### 7.4 VIO triangulation

| Quantity | Config key | Reader | Metric fn | Vis fn |
|---|---|---|---|---|
| Parallax angle | `parallax_angle` | `load("vio_triang")` | `arccos` if `unit=="deg"` | `visualisation.histogram`+`time_series` |
| Baseline distribution | `baseline_distribution` | same | none | `visualisation.histogram` |
| Triangulation rejection funnel | `triangulation_funnel` | same | none | `visualisation.stacked_bar` |

### 7.5 VIO optimisation

| Quantity | Config key | Reader | Metric fn | Vis fn |
|---|---|---|---|---|
| Cost convergence curve | `cost_curve` | `load("vio_linearise")`, `group_by(df,["t_ns"])` | none | `visualisation.line_per_group` |
| Damping trace | `damping_trace` | `load("vio_solver_iter")` | none | `visualisation.time_series` |
| Accept to reject ratio | `accept_reject_ratio` | same | `mean(status==0)` | scalar |
| Cost split | `cost_split` | same | none | `visualisation.multi_axis_time_series`, one subplot per term |
| Solve outcome | `solve_outcome` | `load("vio_solver_summary")` | none | `visualisation.categorical_timeline` |
| Marginalisation size | `marg_size` | `load("vio_marg")` | none | `visualisation.time_series` |
| Marg eigenspectrum | `marg_eigenspectrum` | `load("vio_marg_nullspace")` | none | `visualisation.time_series`+`stacked_bar` |
| Marg nullspace response, named directions | `marg_nullspace_response` | `load("vio_marg_nullspace")`, `vec(df,"marg_ns",7)` | `metrics.nullspace_response(...)`, grounding `[11]` | `visualisation.multi_axis_time_series`, one subplot per direction |
| Pose update acceptance | `pose_update_acceptance` | `load("vio_pose_update")` | none | `visualisation.time_series` |
| Association ratio | `association_ratio` | `load("vio_assoc")` | none | `visualisation.time_series` with `take_kf` marker overlay |
| Landmark churn | `landmark_churn` | `load("vio_landmarks")` | none | `visualisation.time_series` |
| Init sanity | `init_sanity` | `load("vio_init")` | none | scalar, into `metrics.csv` |
| State and divergence trace | `state_trace` | `load("vio_state")` | none | `visualisation.time_series` |
| Excitation covariance | `excitation_covariance` | `load("vio_imu_frame")`, `load("vio_state")`, `load("vio_init")`, joined on `t_ns` | `metrics.align_frames_on_t_ns(...)`, `metrics.excitation_covariance(..., window_s)`, `[8][9][12]` | `visualisation.time_series` |
| Stationarity | `stationarity` | `load("vio_imu_frame")` | `mean(is_stationary)` for the rate metric | `visualisation.categorical_timeline` for the latched flag, `visualisation.multi_axis_time_series` for the two deviation inputs the test is thresholded on |
| Mapper cost curve | `mapper_cost_curve` | `load("mapper_linearise")`, `group_by(df,["t_ns","opt_pass"])` | none | `visualisation.line_per_group` |
| Mapper damping / accept reject | `mapper_damping_accept_reject` | `load("mapper_solver_iter")`, same grouping | none | `visualisation.time_series` |

### 7.6 Mapping detection

| Quantity | Config key | Reader | Metric fn | Vis fn |
|---|---|---|---|---|
| Detector yield | `mapper_detection_yield` | `load("mapper_detect")` | none | `visualisation.time_series` |
| Mapper track length histogram | `mapper_track_length_histogram` | `load("map_tracks")`, `vec(df,"len_hist",10)` | none | `visualisation.histogram_prebinned` |
| Map building throughput | `map_building_throughput` | `load("map_tracks")` | none | `visualisation.time_series` |

### 7.7 Mapping matching

| Quantity | Config key | Reader | Metric fn | Vis fn |
|---|---|---|---|---|
| BoW precision recall curve | `bow_pr_curve` | `load("map_bow_candidate")`, `load("gt_eval")` | `metrics.bow_pr_curve(..., distance_thresh_m, min_time_gap_s)`, `[7][13][14]` | `visualisation.line` |
| BoW score histogram | `bow_score_histogram` | same, reuses the label from `bow_pr_curve` | none | `visualisation.multi_axis_histogram`, true match/false match subplots |
| BoW raw retrieval yield | `bow_raw_retrieval_yield` | `load("map_bow_query")` | none | `visualisation.time_series` |
| Pairwise match funnel | `pairwise_match_funnel` | `load("map_pair_match")` | none | `visualisation.stacked_bar` |
| Matching stage summary | `matching_stage_summary` | `load("map_match_summary")` | none | `visualisation.time_series` |

### 7.8 Mapping triangulation

| Quantity | Config key | Reader | Metric fn | Vis fn |
|---|---|---|---|---|
| Mapper triangulation funnel | `mapper_triangulation_funnel` | `load("map_setup_opt")` | none | `visualisation.stacked_bar` |

### 7.9 Mapping optimisation and map quality

| Quantity | Config key | Reader | Metric fn | Vis fn |
|---|---|---|---|---|
| Covisibility graph | `covisibility_graph` | `load("map_covis")` | none | `visualisation.heatmap`, adjacency proxy rather than a graph layout, see section 9 |
| Culling redundancy calibration | `culling_redundancy_calibration` | `load("map_cull_select")`, `vec(df,"observers_hist",10)` | none | `visualisation.histogram_prebinned` |
| Keyframe culling rate | `keyframe_culling_rate` | `load("map_cull")`, `load("map_cull_select")` | `culled/created` | scalar |
| Landmark filter attrition | `landmark_filter_attrition` | `load("map_filter")` | none | `visualisation.time_series` |
| Publish consistency | `publish_consistency` | `load("map_publish")` | none | `visualisation.time_series` |
| Landmark rehost funnel | `landmark_rehost_funnel` | `load("map_rehost")` | `df.groupby(["outcome","reason"]).size()` | `visualisation.stacked_bar` |
| Culled keyframe summary | `culled_keyframe_summary` | `load("map_cull")` | none | `visualisation.time_series` |

### 7.10 Timing and systems

| Quantity | Config key | Reader | Metric fn | Vis fn |
|---|---|---|---|---|
| Front end stage latency | `frontend_stage_latency` | `load("of_timing")` | none | `visualisation.multi_axis_time_series`, one subplot per stage |
| VIO stage latency | `vio_stage_latency` | `load("vio_timing")` | none | `visualisation.multi_axis_time_series`, one subplot per stage |
| VIO queue depth | `vio_queue_depth` | same | none | `visualisation.time_series` |
| VIO solver stage latency | `vio_solver_stage_latency` | `load("vio_solver_timing")` | none | `visualisation.multi_axis_time_series`, one subplot per stage |
| VIO marg stage latency | `vio_marg_stage_latency` | `load("vio_marg_timing")` | none | `visualisation.multi_axis_time_series`, one subplot per stage |
| Mapper cycle latency | `mapper_cycle_latency` | `load("map_cycle_timing")` | `queue_wait/total` | `visualisation.multi_axis_time_series`, one subplot per stage |
| Mapper detection latency | `mapper_detection_latency` | `load("mapper_detect")` | none | `visualisation.time_series` |
| Mapper matching latency | `mapper_matching_latency` | `load("map_match_summary")` | none | `visualisation.multi_axis_time_series`, `db_query`/`matching` subplots |
| IMU ingestion health | `imu_ingestion_health` | `load("vio_imu_ingest")` | none | `visualisation.time_series` |
| IMU frame coverage | `imu_frame_coverage` | `load("vio_imu_frame")` | none | `visualisation.time_series` |
| Tail latency percentiles | `tail_latency_percentiles` | every timing block above, uniformly, via `log_stage_scalars` | `np.percentile(...,[95,99])` | scalar table |

### 7.11 Cross run

| Quantity | Config key | Reader | Metric fn | Vis fn |
|---|---|---|---|---|
| Config diff | `config_diff` | `load_json("config")`, `load_json("calibration")` | `metrics.config_diff(configs, baseline)` | `visualisation.diff_table` |
| Distribution comparison | `distribution_comparison` | whichever file the named metric reads | none, reuses that metric's own values | `visualisation.box_violin` |

Section 7 of `context/slam_analysis.md` is NEES, NIS and the covariance ellipsoid, all three explicitly deferred there, needing a per state covariance this stack does not yet extract, and left out of `config.json` entirely rather than kept as a disabled placeholder block, since a key with no implementation behind it is dead configuration surface regardless of whether it is enabled or not. The open items list `context/slam_analysis.md` section 10 already keeps is the one place a reader looking for these three finds them.

## 8. metrics.py and visualisation.py, the shared functions

`metrics.py` holds every function the routing table names once, `TrajectoryAnalysis.rpe`'s own `relative_pose_error`/`relative_pose_error_sweep`/`_invert_se3_batch` moved here verbatim from `context/slam_analysis.md` section 2.2, plus the genuinely bespoke joins. `combine_ground_truth` builds the single ground truth trajectory the 3D overlay plot needs by taking the union of every experiment's own ground truth samples and deduplicating on `t_ns`, which only coincide frame for frame because `DataReader` has already offset every experiment's logs, this one included, by that experiment's own mission start, section 5's `_scan_epochs`, so no further shift is needed here. `align_frames_on_t_ns` is the general answer to the same alignment question between two different logging sites within one experiment rather than between two experiments, an inner merge on the shared `t_ns` column rather than a positional zip truncated to `min(len(left), len(right))`, since the two sites need not drop exactly the same frame and a positional pairing silently pairs unrelated frames the moment one site is missing a row the other kept, `excitation_covariance`'s own join of the IMU frame record against the state trace in section 9 being the one place this stack currently needs it. `pivot_metrics` pivots on `["category", "name", "type"]` rather than a single `"metric"` column, `name` a human phrase such as `"ATE (m)"`, `type` the statistic kind such as `"rmse"`, section 9 below is where every call site supplies both.

```python
# metrics.py
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

try:
    import cupy as cp
    _HAVE_CUPY = True
except ImportError:
    cp = np
    _HAVE_CUPY = False


def _invert_se3_batch(T, xp):
    """T: (N, 4, 4) -> (N, 4, 4). R is orthonormal by construction, so
    T^-1 = [[R^T, -R^T t], [0, 1]] rather than a general xp.linalg.inv."""
    R = T[:, :3, :3]
    t = T[:, :3, 3:4]
    Rt = xp.swapaxes(R, 1, 2)
    Tinv = xp.zeros_like(T)
    Tinv[:, :3, :3] = Rt
    Tinv[:, :3, 3:4] = -xp.matmul(Rt, t)
    Tinv[:, 3, 3] = 1.0
    return Tinv


def relative_pose_error(T_est, T_gt, delta, xp=None, translation_only=True):
    """Timestamp wise Relative Pose Error at one fixed sub trajectory length,
    batched into one array operation per delta rather than a Python loop over
    pairs. T_est, T_gt must already be ALIGNED SE(3) poses, (N, 4, 4) each.

    Returns
        per_start_error : (N - delta,) array, entry i is the RPE of the sub
            trajectory starting at frame i, aligned one to one with
            t_ns[:-delta].
        rmse : scalar RMSE over per_start_error.
    """
    xp = xp or (cp if _HAVE_CUPY else np)
    T_est = xp.asarray(T_est)
    T_gt = xp.asarray(T_gt)
    n = T_est.shape[0]
    i0 = xp.arange(0, n - delta)
    i1 = i0 + delta
    rel_est = xp.matmul(_invert_se3_batch(T_est[i0], xp), T_est[i1])
    rel_gt = xp.matmul(_invert_se3_batch(T_gt[i0], xp), T_gt[i1])
    E = xp.matmul(_invert_se3_batch(rel_gt, xp), rel_est)
    if translation_only:
        per_start_error = xp.linalg.norm(E[:, :3, 3], axis=1)
    else:
        trace = xp.clip((E[:, 0, 0] + E[:, 1, 1] + E[:, 2, 2] - 1.0) / 2.0, -1.0, 1.0)
        per_start_error = xp.arccos(trace)
    rmse = xp.sqrt(xp.mean(per_start_error ** 2))
    return per_start_error, rmse


def relative_pose_error_sweep(T_est, T_gt, deltas, xp=None):
    """Runs relative_pose_error for every delta, returning the RPE vs sub
    trajectory length curve and every delta's own timestamp wise vector."""
    xp = xp or (cp if _HAVE_CUPY else np)
    curve = []
    per_delta_series = {}
    for delta in deltas:
        per_start_error, rmse = relative_pose_error(T_est, T_gt, delta, xp=xp)
        curve.append((delta, float(rmse)))
        per_delta_series[delta] = cp.asnumpy(per_start_error) if (_HAVE_CUPY and xp is cp) else np.asarray(per_start_error)
    return curve, per_delta_series


def rpe_curve_with_band(per_delta_series: dict[int, np.ndarray]):
    """Mean and standard deviation band per length from relative_pose_error_sweep's
    own per_delta_series. Returns deltas, mean error per delta, std error per
    delta, in delta order."""
    deltas = sorted(per_delta_series)
    means = np.array([per_delta_series[d].mean() for d in deltas])
    stds = np.array([per_delta_series[d].std() for d in deltas])
    return np.array(deltas), means, stds


def combine_ground_truth(t_ns_per_run: dict[str, np.ndarray],
                          gt_xyz_per_run: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Every experiment replays the same mission, and DataReader has already
    offset every one of its logs, this one included, by that experiment's
    own mission start (see DataReader._scan_epochs), so the ground truth
    samples here already share one zero point and coincide frame for frame
    without any further shift. The union across every experiment,
    deduplicated on that now-comparable timestamp, is a single trajectory
    covering every frame any one experiment reached."""
    t_all = np.concatenate([np.asarray(t_ns) for t_ns in t_ns_per_run.values()])
    xyz_all = np.concatenate([gt_xyz_per_run[exp] for exp in t_ns_per_run])
    _, idx = np.unique(t_all, return_index=True)
    return t_all[idx], xyz_all[idx]


def align_frames_on_t_ns(left: pd.DataFrame, right: pd.DataFrame) -> pd.DataFrame:
    """Inner merge of two per frame logs on their shared t_ns column, the
    only safe way to line up two logging sites that need not drop exactly
    the same frames. Truncating both to min(len(left), len(right)) and
    pairing by row position instead silently pairs unrelated frames the
    moment either site is missing a row the other kept, which is exactly
    what one logging site dropping a different frame than the other
    produces."""
    return pd.merge(left, right, on="t_ns", how="inner").sort_values("t_ns").reset_index(drop=True)


def drift_proxies(gt_xyz: np.ndarray, gt_qxyzw: np.ndarray,
                   est_xyz: np.ndarray, est_qxyzw: np.ndarray) -> dict:
    """closure_error_m is the distance between the first and last estimated
    position minus the same distance in ground truth, near zero for a mission
    that lands where it took off regardless of absolute scale. A second,
    disposable TrajectoryAnalysis("sim3") instance fits a full similarity
    transform over the whole trajectory, independent of whatever
    alignment_mode is configured for ate()/rpe() elsewhere, and its scale,
    rotation and translation are reported directly as scale_drift/
    rotation_drift_deg/translation_drift_m. A perfect estimate reports
    scale_drift near 1.0 and the other two near zero. Imported locally
    because trajectory_analysis.py imports relative_pose_error_sweep from
    this module; a module level import here would close that into a cycle."""
    from trajectory_analysis import TrajectoryAnalysis
    est_closure = np.linalg.norm(est_xyz[-1] - est_xyz[0])
    gt_closure = np.linalg.norm(gt_xyz[-1] - gt_xyz[0])
    ta_sim3 = TrajectoryAnalysis("sim3")
    ta_sim3.set_alignment(gt_xyz, gt_qxyzw, est_xyz, est_qxyzw)
    rotation_drift_deg = float(Rotation.from_matrix(ta_sim3.R).magnitude() * 180.0 / np.pi)
    out = {
        "closure_error_m": float(est_closure - gt_closure),
        "scale_drift": float(ta_sim3.s),
        "rotation_drift_deg": rotation_drift_deg,
        "translation_drift_m": float(np.linalg.norm(ta_sim3.t)),
    }
    ta_sim3.reset_alignment()
    return out


def mean_std_scalars(df: pd.DataFrame, cols: list[str]) -> dict[str, tuple[float, float]]:
    return {c: (float(df[c].mean()), float(df[c].std())) for c in cols if c in df}


def accept_reject_ratio(status: pd.Series) -> float:
    """status 0 accepted, 1 rejected or invalid."""
    return float((status == 0).mean())


def inlier_outlier_ratio(tracked: pd.Series, epi_rej: pd.Series) -> np.ndarray:
    denom = (tracked + epi_rej).replace(0, np.nan)
    return (tracked / denom).to_numpy()


def value_counts_dict(series: pd.Series) -> dict:
    return series.value_counts().sort_index().to_dict()


def keyframe_culling_rate(map_cull_df: pd.DataFrame) -> float:
    return float(map_cull_df["victims"].sum() / max(map_cull_df["victims"].sum() + len(map_cull_df), 1))


def mapper_idle_fraction(map_cycle_timing_df: pd.DataFrame) -> np.ndarray:
    return (map_cycle_timing_df["queue_wait"] / map_cycle_timing_df["total"]).to_numpy()


def tail_percentiles(df: pd.DataFrame, cols: list[str], percentiles: list[float]) -> dict:
    out = {}
    for c in cols:
        if c not in df:
            continue
        values = np.percentile(df[c].dropna(), percentiles)
        for p, v in zip(percentiles, values):
            out[f"{c}_p{int(p)}"] = float(v)
    return out


def pivot_metrics(metrics_rows: list[dict]) -> pd.DataFrame:
    """metrics_rows is long form, one row per (category, experiment, name,
    type, value), name a human phrase ("ATE"), type the statistic kind
    ("rmse", "mean", "percentile", ...), the two split so two rows can share
    a name and differ only by type. Pivoted here to one row per (category,
    name, type), one column per experiment, the shape both metrics.csv and
    the scalars tab need."""
    if not metrics_rows:
        return pd.DataFrame(columns=["category", "name", "type"])
    long = pd.DataFrame(metrics_rows)
    wide = long.pivot(index=["category", "name", "type"], columns="experiment", values="value")
    return wide.reset_index()


def survival_curve(age_frames: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ages = np.sort(age_frames)
    n = ages.size
    grid = np.arange(ages.max() + 1)
    survived = n - np.searchsorted(ages, grid, side="right")
    return grid, survived / n


def bow_pr_curve(candidates: pd.DataFrame, gt_eval: pd.DataFrame,
                  distance_thresh_m: float, min_time_gap_s: float):
    """candidates carries query_kf/candidate_kf/score/above_thresh; both ids
    are themselves frame timestamps in this stack's t_ns domain, so gt_eval
    is joined by nearest t_ns for each."""
    gt = gt_eval.dropna(subset=[c for c in gt_eval.columns if c.startswith("p_gt")])
    gt = gt.sort_values("t_ns")

    def pos_at(t_ns):
        idx = np.searchsorted(gt["t_ns"].to_numpy(), t_ns)
        idx = np.clip(idx, 0, len(gt) - 1)
        return gt.iloc[idx][["p_gt_0", "p_gt_1", "p_gt_2"]].to_numpy()

    p_query = np.stack(candidates["query_kf"].map(pos_at))
    p_cand = np.stack(candidates["candidate_kf"].map(pos_at))
    dist = np.linalg.norm(p_query - p_cand, axis=1)
    dt_s = np.abs(candidates["query_kf"] - candidates["candidate_kf"]) * 1e-9
    is_true = (dist < distance_thresh_m) & (dt_s > min_time_gap_s)
    thresholds = np.sort(candidates["score"].unique())
    precision, recall = [], []
    total_true = is_true.sum()
    for t in thresholds:
        pred = candidates["score"] >= t
        tp = (pred & is_true).sum()
        precision.append(tp / max(pred.sum(), 1))
        recall.append(tp / max(total_true, 1))
    return np.array(recall), np.array(precision), is_true


def nullspace_response(marg_ns: np.ndarray) -> dict[str, np.ndarray]:
    """marg_ns, an (N, 7) array from vec(df, "marg_ns", 7), is already the per
    direction xHx + xb probe checkNullspace computes (src/vi_estimator/
    sqrt_ba_base.cpp:43-227) for six candidate global perturbations, a world
    frame translation shift along x, y, z and a world frame rotation shift
    about x (roll), y (pitch), z (yaw), plus a seventh, deliberately
    unconstrained random direction kept as a scale reference. Nothing is
    projected here, the seven components already are the per axis answer."""
    names = ["trans_x", "trans_y", "trans_z", "roll", "pitch", "yaw", "random_control"]
    return {name: marg_ns[:, i] for i, name in enumerate(names)}


def excitation_covariance(accel_mean_world: np.ndarray, window_s: float, dt_s: float) -> np.ndarray:
    """Windowed sample covariance of the unit gravity compensated specific
    force direction. accel_mean_world is already gravity subtracted and
    world rotated by the caller."""
    window = max(int(window_s / dt_s), 3)
    unit = accel_mean_world / np.linalg.norm(accel_mean_world, axis=1, keepdims=True)
    out = np.full(len(unit), np.nan)
    for i in range(window, len(unit)):
        cov = np.cov(unit[i - window:i].T)
        out[i] = np.linalg.eigvalsh(cov).min()
    return out


def _flatten(d: dict, prefix: str = "") -> dict:
    """config.json/calibration.json are cereal JSON, wrapped one level deep
    as {"value0": {...}}, so config_diff must flatten this first or every
    comparison reads as one opaque difference on the single key "value0"."""
    out = {}
    for k, v in d.items():
        key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            out.update(_flatten(v, key))
        else:
            out[key] = v
    return out


def config_diff(configs: dict[str, dict], baseline: str) -> pd.DataFrame:
    """Flat key comparison of every experiment's config against one baseline
    experiment, one row per differing leaf field."""
    base = _flatten(configs[baseline])
    rows = []
    for exp, cfg in configs.items():
        flat = _flatten(cfg)
        for key in base:
            if flat.get(key) != base[key]:
                rows.append({"experiment": exp, "key": key, "baseline": base[key], "value": flat.get(key)})
    return pd.DataFrame(rows)
```

`visualisation.py` holds the plot shapes, each a thin `plotly.graph_objects` builder taking the data already reduced to plain arrays or DataFrames by `metrics.py` or by `class Analysis` itself, one experiment's own series distinguished by colour and named in the legend so Plotly's own legend click toggles it.

Every function below takes and sets a `title`, an `xaxis_title` and a `yaxis_title` (or `scene` axis titles for the 3D plots) explicitly, never left at Plotly's own blank default, and every trace is `name`d per experiment with `legend_title_text` set, so Plotly's own legend, already showing since every trace is named, reads "Experiment" rather than an unlabelled key list. Every function also takes a trailing `width`/`height` pair, passed straight to `fig.update_layout` by the shared `_size` helper. `class Analysis`'s own `_wh` helper reads these two keys out of whichever `config.json` entry produced the figure and splats them in, so any one plot's size is configurable without a code change. `config.json`, section 11, sets both on every plot that has one, sized for a 1440px wide viewport, the common logical resolution a MacBook screen reports, at either three quarters width (1080px, a single axis plot whose x axis needs the room, or a multi subplot figure) or half width (720px, a histogram, curve or bar chart with no long x axis to stretch), with a height chosen for what that plot's own shape reads best at rather than one fixed number dashboard wide, a wide, short aspect for a long running time series, a more square one for a distribution or funnel, tall enough for every row of a multi subplot figure, and 1080x810 for the two 3D trajectory plots.

`_size` also takes `link_zoom`, tagging a figure's `layout.meta.linked_zoom` when set. `time_series`, `multi_axis_time_series` and `categorical_timeline` are the three functions that ever set it true, and then only at call sites in `class Analysis` whose x axis is a genuine shared time base, `t_ns`, never a plot's own arbitrary index such as a global solver iteration count. `dashboard.py`, section 10, reads this flag back out of the saved page's own Plotly figures client side and links every tagged figure's zoom and pan together, so this flag, not `make_subplots(shared_xaxes=True)` alone, is what makes zooming into a time window on any one linked plot, anywhere on the page, zoom every other linked plot to that same window. `make_subplots(shared_xaxes=True)`, used by `multi_axis_time_series`/`multi_axis_histogram` below, still separately links every subplot's own axis within one figure to each other, the two mechanisms compose, a multi axis figure's `link_zoom=True` links its own already mutually synced subplots to every other figure on the page as one unit.

`time_series`, `multi_axis_time_series` and `categorical_timeline` plot every run as a marker only trace, `mode="markers"`, never a connected line. Different experiments do not share a sampling grid, since each is its own simulation run with its own frame timing, and even two logging sites within one experiment, the IMU frame record and the state trace for instance, need not drop the same frame, so a line drawn between two consecutive available samples spans whatever gap sits between them and reads as continuous, interpolated data exactly where none was logged. A marker is drawn only at a timestamp that run's own data actually has a row for, so a dropped frame is simply a gap in the marker density rather than something a line silently bridges, and Plotly's own autorange already spans the union of every trace's own x extent, so a run that ends early is not clipped by one that runs longer.

All three use `go.Scattergl`, not `go.Scatter`, for this marker trace. A mission runs to several thousand frames, and Plotly's SVG renderer, what a plain `Scatter` trace uses, draws one `<path>` DOM node per marker, so a dashboard with dozens of such plots across several experiments produces a page with enough SVG nodes to make opening it in a browser, Firefox in particular, noticeably slow, an earlier draft of these three functions used `go.Scatter` and hit exactly this. `Scattergl` rasterises the same markers onto one WebGL canvas per trace instead, whose cost does not grow with DOM node count, which is the fix. The two functions that stay on `go.Scatter`, `line` and `line_with_band`, are unaffected, since neither plots against a per frame time base at mission length, and `line_with_band`'s `fill="toself"` std band additionally needs SVG fill semantics `Scattergl` does not support.

Two functions, `multi_axis_time_series` and `multi_axis_histogram`, use `plotly.subplots.make_subplots` rather than a single axis. Both exist for the same reason, several related quantities, cost split terms, marginalisation nullspace directions, the four to thirteen named stages of a latency breakdown, previously shared one axis as one trace per (experiment, quantity) pair, which stopped scaling once more than a couple of experiments were compared, since the legend and the overlapping traces both grew with the product of experiment count and quantity count. One subplot per quantity, one trace per experiment within it, fixes both problems at once, an experiment is now always compared against itself on a single, uncluttered axis.

```python
# visualisation.py
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

LEGEND_TITLE = "Experiment"


def _size(fig: go.Figure, width: int | None, height: int | None, link_zoom: bool = False) -> go.Figure:
    """link_zoom tags a figure's layout.meta for dashboard.py's own post save
    <script> injection, which finds every tagged figure by this flag and
    wires their x axis zoom/pan together client side, Plotly's own public
    plotly_relayout event and Plotly.relayout, no Panel/Bokeh internals."""
    if link_zoom:
        fig.update_layout(meta={"linked_zoom": True})
    return fig.update_layout(width=width, height=height)


def time_series(t_ns_per_run: dict[str, list], y_per_run: dict[str, list], title: str,
                 xaxis_title: str, yaxis_title: str, log_y: bool = False,
                 marker_index_per_run: dict[str, np.ndarray] | None = None,
                 link_zoom: bool = False, width: int | None = None, height: int | None = None) -> go.Figure:
    """Each run is its own marker only trace at its own sample instants, not
    a connected line, because different experiments and different logging
    sites within one experiment do not share a sampling grid, a dropped
    frame in only one of them is common, and a line drawn across a gap
    implies a value was interpolated there when none was logged. A marker
    simply is not drawn at a timestamp that run has no row for. Scattergl,
    not Scatter, because a mission runs to several thousand frames and a
    plain Scatter marker trace renders one SVG node per point, which is
    what made the saved page slow to open; Scattergl rasterises the same
    markers onto one WebGL canvas per trace regardless of point count.
    marker_index_per_run optionally overlays a second, marker only trace at
    the given indices. link_zoom is set only where the x axis is a genuine
    shared time base, t_ns, not an arbitrary index such as a global
    iteration count another quantity's plot might use."""
    fig = go.Figure()
    for run, y in y_per_run.items():
        fig.add_trace(go.Scattergl(x=t_ns_per_run[run], y=y, name=run, mode="markers",
                                    marker=dict(size=4)))
        if marker_index_per_run and run in marker_index_per_run:
            idx = marker_index_per_run[run]
            fig.add_trace(go.Scattergl(x=np.asarray(t_ns_per_run[run])[idx], y=np.asarray(y)[idx],
                                        name=f"{run} marker", mode="markers"))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title=yaxis_title,
                       yaxis_type="log" if log_y else "linear", legend_title_text=LEGEND_TITLE)
    return _size(fig, width, height, link_zoom)


def multi_axis_time_series(t_ns_per_run: dict[str, list], quantities: dict[str, dict[str, list]],
                            title: str, xaxis_title: str, yaxis_title: str, link_zoom: bool = False,
                            width: int | None = None, height: int | None = None) -> go.Figure:
    """One subplot per quantity, one trace per experiment within it, so
    experiments compare directly within an axis and quantities compare by
    reading down the figure. shared_xaxes links every subplot's zoom/pan to
    the bottom axis within this figure; link_zoom additionally links this
    figure's own x axis to every other linked figure on the page. Markers
    only, the same reasoning as time_series, since the quantities compared
    here routinely come from different logging sites that do not share a
    sampling grid, and Scattergl rather than Scatter for the same reason,
    one WebGL canvas per trace rather than one SVG node per point."""
    names = list(quantities)
    fig = make_subplots(rows=len(names), cols=1, shared_xaxes=True,
                         subplot_titles=names, vertical_spacing=0.03)
    for i, name in enumerate(names, start=1):
        for run, y in quantities[name].items():
            fig.add_trace(go.Scattergl(x=t_ns_per_run[run], y=y, name=run, mode="markers",
                                        marker=dict(size=4), legendgroup=run,
                                        showlegend=(i == 1)), row=i, col=1)
    fig.update_xaxes(title_text=xaxis_title, row=len(names), col=1)
    fig.update_yaxes(title_text=yaxis_title)
    fig.update_layout(title=title, legend_title_text=LEGEND_TITLE)
    return _size(fig, width, height or min(300 * len(names), 2400), link_zoom)


def multi_axis_histogram(quantities: dict[str, dict[str, list]], title: str, xaxis_title: str,
                          width: int | None = None, height: int | None = None) -> go.Figure:
    """One subplot per quantity, one histogram trace per experiment within it."""
    names = list(quantities)
    fig = make_subplots(rows=len(names), cols=1, subplot_titles=names, vertical_spacing=0.1)
    for i, name in enumerate(names, start=1):
        for run, values in quantities[name].items():
            fig.add_trace(go.Histogram(x=values, name=run, opacity=0.6,
                                        legendgroup=run, showlegend=(i == 1)), row=i, col=1)
    fig.update_xaxes(title_text=xaxis_title, row=len(names), col=1)
    fig.update_yaxes(title_text="count")
    fig.update_layout(title=title, barmode="overlay", legend_title_text=LEGEND_TITLE)
    return _size(fig, width, height or min(350 * len(names), 2400))


def line(x_per_run: dict[str, list], y_per_run: dict[str, list], title: str,
         xaxis_title: str, yaxis_title: str, width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure()
    for run, y in y_per_run.items():
        fig.add_trace(go.Scatter(x=x_per_run[run], y=y, name=run, mode="lines+markers"))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title=yaxis_title,
                       legend_title_text=LEGEND_TITLE)
    return _size(fig, width, height)


def line_with_band(x_per_run: dict[str, np.ndarray], mean_per_run: dict[str, np.ndarray],
                    std_per_run: dict[str, np.ndarray], title: str, xaxis_title: str,
                    yaxis_title: str, width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure()
    for run in mean_per_run:
        x, mean, std = np.asarray(x_per_run[run]), np.asarray(mean_per_run[run]), np.asarray(std_per_run[run])
        fig.add_trace(go.Scatter(x=x, y=mean, name=run, mode="lines+markers"))
        fig.add_trace(go.Scatter(x=np.concatenate([x, x[::-1]]),
                                  y=np.concatenate([mean + std, (mean - std)[::-1]]),
                                  fill="toself", opacity=0.2, line=dict(width=0),
                                  name=f"{run}, +/- 1 std", showlegend=False))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title=yaxis_title,
                       legend_title_text=LEGEND_TITLE)
    return _size(fig, width, height)


def histogram(values_per_run: dict[str, list], title: str, xaxis_title: str, bins: int = 30,
              width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure()
    for run, values in values_per_run.items():
        fig.add_trace(go.Histogram(x=values, name=run, nbinsx=bins, opacity=0.6))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title="count",
                       barmode="overlay", legend_title_text=LEGEND_TITLE)
    return _size(fig, width, height)


def histogram_prebinned(bin_labels: list[str], counts_per_run: dict[str, list], title: str,
                         xaxis_title: str, width: int | None = None, height: int | None = None) -> go.Figure:
    """Histograms already binned at logging time, whose bin edges are fixed
    by the C++ logger, hence a grouped bar rather than go.Histogram."""
    fig = go.Figure()
    for run, counts in counts_per_run.items():
        fig.add_trace(go.Bar(x=bin_labels, y=counts, name=run))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title="count",
                       barmode="group", legend_title_text=LEGEND_TITLE)
    return _size(fig, width, height)


def stacked_bar(counts_per_run: dict[str, dict], title: str, xaxis_title: str = "category",
                 width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure()
    for run, counts in counts_per_run.items():
        fig.add_trace(go.Bar(x=list(counts.keys()), y=list(counts.values()), name=run))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title="count",
                       barmode="stack", legend_title_text=LEGEND_TITLE)
    return _size(fig, width, height)


def categorical_timeline(t_ns_per_run: dict[str, list], category_per_run: dict[str, list],
                          title: str, xaxis_title: str, link_zoom: bool = False,
                          width: int | None = None, height: int | None = None) -> go.Figure:
    """One marker trace per category value per run; y axis is the run name
    so several runs stack as separate rows. Scattergl, not Scatter, since
    one of these runs to several thousand points and a plain SVG marker
    trace renders one DOM node per point."""
    fig = go.Figure()
    for run, categories in category_per_run.items():
        categories = np.asarray(categories)
        for value in sorted(set(categories.tolist())):
            mask = categories == value
            fig.add_trace(go.Scattergl(x=np.asarray(t_ns_per_run[run])[mask],
                                        y=[run] * int(mask.sum()), mode="markers",
                                        name=f"{run}, {value}"))
    fig.update_layout(title=title, xaxis_title=xaxis_title, yaxis_title="experiment",
                       legend_title_text=LEGEND_TITLE)
    return _size(fig, width, height, link_zoom)


def line_3d(xyz_per_run: dict[str, np.ndarray], title: str,
            colour_per_run: dict[str, np.ndarray] | None = None,
            width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure()
    for run, xyz in xyz_per_run.items():
        marker = dict(color=colour_per_run[run], colorscale="Viridis", size=2,
                       colorbar=dict(title="ATE (m)")) if colour_per_run else None
        fig.add_trace(go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2], name=run,
                                    mode="lines" if marker is None else "markers", marker=marker))
    fig.update_layout(title=title, legend_title_text=LEGEND_TITLE,
                       scene=dict(xaxis_title="x (m)", yaxis_title="y (m)", zaxis_title="z (m)"))
    return _size(fig, width, height)


def trajectory_3d_overlay(gt_xyz: np.ndarray, est_xyz_per_run: dict[str, np.ndarray], title: str,
                           width: int | None = None, height: int | None = None) -> go.Figure:
    """gt_xyz is the single combined ground truth trajectory, always drawn
    green; est_xyz_per_run gets one line per experiment in Plotly's own
    default per-trace colour cycle."""
    fig = go.Figure()
    fig.add_trace(go.Scatter3d(x=gt_xyz[:, 0], y=gt_xyz[:, 1], z=gt_xyz[:, 2],
                                name="ground truth", mode="lines", line=dict(color="green", width=4)))
    for run, xyz in est_xyz_per_run.items():
        fig.add_trace(go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2], name=run, mode="lines"))
    fig.update_layout(title=title, legend_title_text=LEGEND_TITLE,
                       scene=dict(xaxis_title="x (m)", yaxis_title="y (m)", zaxis_title="z (m)"))
    return _size(fig, width, height)


def heatmap(matrix_per_run: dict[str, np.ndarray], title: str, width: int | None = None,
            height: int | None = None) -> go.Figure:
    """One run at a time, a heatmap has no legend of its own to distinguish
    overlays."""
    run, matrix = next(iter(matrix_per_run.items()))
    fig = go.Figure(go.Heatmap(z=matrix, colorbar=dict(title="shared landmarks")))
    fig.update_layout(title=f"{title}, {run}", xaxis_title="keyframe index", yaxis_title="keyframe index")
    return _size(fig, width, height)


def diff_table(diff_df, title: str, width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure(go.Table(
        header=dict(values=list(diff_df.columns)),
        cells=dict(values=[diff_df[c] for c in diff_df.columns]),
    ))
    fig.update_layout(title=title)
    return _size(fig, width, height)


def box_violin(values_per_run: dict[str, list], title: str, yaxis_title: str, kind: str = "box",
               width: int | None = None, height: int | None = None) -> go.Figure:
    fig = go.Figure()
    trace = go.Box if kind == "box" else go.Violin
    for run, values in values_per_run.items():
        fig.add_trace(trace(y=values, name=run))
    fig.update_layout(title=title, yaxis_title=yaxis_title, legend_title_text=LEGEND_TITLE)
    return _size(fig, width, height)
```

## 9. class Analysis

Every scalar computed anywhere in `class Analysis`, not only the handful section 7's table calls out by name, is appended through `self._add_metric`, the one and only path into `self.metrics_rows`. `run()` writes that list, pivoted, to `metrics.csv`; `dashboard.py` pivots the same list again for the scalars tab, section 10, so the CSV and the on screen table are always the complete and only scalar record a run of `dashboard.py` produces. No scalar is ever rendered as its own tile shaped figure inside a subsystem tab, drift proxies and initialisation sanity readings included, every one of them is a row in the scalars table instead, so a reader has exactly one place to look for every number this dashboard computes.

`self._add_metric(category, experiment, name, type_, value)` takes `name` and `type_` as two separate arguments rather than one flat metric string, `name` a human phrase with no underscores, `"ATE (m)"` rather than `"ate_rmse_m"`, `type_` the statistic kind, `"rmse"`/`"mean"`/`"std"`/`"percentile"`/`"rate"`/`"value"`, so two rows can share a name and differ only by type, `"ATE (m)"` at `"rmse"` and again at `"mean"` for instance. Two module level lookup tables, `DRIFT_PROXY_LABELS` and `INIT_SANITY_LABELS`, hold the fixed name/type pairs the trajectory and VIO optimisation tabs need for their own few scalar only quantities. Every timing bearing CSV additionally has its own mean, standard deviation and, where `timing.tail_latency_percentiles` is enabled, tail percentiles added per named stage through one shared closure, `log_stage_scalars`, defined once inside `_analyse_timing` and called from every timing block in that method, front end, VIO, VIO solver, VIO marginalisation, mapper cycle, mapper detection and mapper matching alike, so a stage that is only ever shown as a time series still has a comparable scalar record in the table, for every timing quantity the dashboard logs, not a hand picked subset. `log_stage_scalars` turns a `(prefix, stage)` pair such as `("vio_solver", "performQR")` into a phrase, `"VIO solver QR factorisation latency"`, via two more module level lookup tables, `STAGE_PREFIX_LABELS` and `STAGE_LABELS`, rather than passing the raw snake_case or camelCase CSV column name through unchanged.

`self._wh(cfg_entry)` is the one place `width`/`height` are read out of a `config.json` entry and turned into the keyword arguments every `visualisation.py` function accepts, splatted in at each call site with `**self._wh(cfg["<key>"])`. `link_zoom=True` is passed alongside it at every call site whose x axis is `t_ns`, a plain keyword argument on the `visualisation.py` call rather than anything read from `config.json`, since which plots share a time axis is a property of the plot, not a per run choice.

```python
# analysis.py
import os

import metrics
import numpy as np
from data_reader import DataReader
from scipy.spatial.transform import Rotation
from trajectory_analysis import TrajectoryAnalysis

import visualisation

TABS = [
    "trajectory",
    "optical_flow_detection",
    "optical_flow_tracking",
    "vio_triangulation",
    "vio_optimisation",
    "mapping_detection",
    "mapping_matching",
    "mapping_triangulation",
    "mapping_optimisation",
    "timing",
    "cross_run",
]

DRIFT_PROXY_LABELS = {
    "closure_error_m": ("Closure error (m)", "value"),
    "scale_drift": ("Scale drift", "value"),
    "rotation_drift_deg": ("Rotation drift (deg)", "value"),
    "translation_drift_m": ("Translation drift (m)", "value"),
}

INIT_SANITY_LABELS = {
    "tilt_deg": "Initial tilt (deg)",
    "accel_norm": "Initial accelerometer norm",
    "gyro_norm": "Initial gyroscope norm",
}

STAGE_PREFIX_LABELS = {
    "of": "Front end",
    "vio": "VIO",
    "vio_solver": "VIO solver",
    "vio_marg": "VIO marginalisation",
    "mapper": "Mapper cycle",
    "mapper_detect": "Mapper detection",
    "mapper_matching": "Mapper matching",
}

STAGE_LABELS = {
    "pyramid": "pyramid build",
    "track": "tracking",
    "detect_add": "detection and insertion",
    "total": "total",
    "pose_update": "pose update",
    "association": "association",
    "triangulation": "triangulation",
    "lost_scan": "lost scan",
    "opt_and_marg": "optimisation and marginalisation",
    "publish": "publish",
    "measure": "measurement",
    "process_frame": "frame processing",
    "allocateLMB": "landmark block allocation",
    "linearizeProblem": "linearisation",
    "performQR": "QR factorisation",
    "get_dense_H_b": "dense H/b assembly",
    "solve": "solve",
    "backSubstitute": "back substitution",
    "computerError2": "error evaluation",
    "marg_linearize": "marginalisation linearisation",
    "marg_helper": "marginalisation helper",
    "marg": "marginalisation",
    "marg_log": "marginalisation logging",
    "ingest": "ingest",
    "detect_keypoints": "keypoint detection",
    "match_stereo": "stereo matching",
    "match_local": "local matching",
    "collect_new_kfs": "new keyframe collection",
    "build_tracks": "track building",
    "setup_opt": "optimisation setup",
    "cull": "culling",
    "optimize_1": "first optimisation pass",
    "filter_outliers": "outlier filtering",
    "optimize_2": "second optimisation pass",
    "publish_vis": "visualisation publish",
    "pose_update_cb": "pose update callback",
    "queue_wait": "queue wait",
    "detection": "detection",
    "db_query": "database query",
    "matching": "matching",
}


class Analysis:
    def __init__(self, reader: DataReader, config: dict, out_dir: str):
        self.reader, self.config, self.out_dir = reader, config, out_dir
        self.metrics_rows: list[dict] = []
        self.tabs: dict[str, list[tuple[str, object]]] = {}

    def run(self):
        for tab in TABS:
            method = getattr(self, f"_analyse_{tab}")
            self.tabs[tab] = method(self.config.get(tab, {}))
        os.makedirs(os.path.join(self.out_dir, "figures"), exist_ok=True)
        for tab, figs in self.tabs.items():
            tab_dir = os.path.join(self.out_dir, "figures", tab)
            os.makedirs(tab_dir, exist_ok=True)
            for title, fig in figs:
                fig.write_html(
                    os.path.join(tab_dir, f"{title}.html"), include_plotlyjs=True
                )
        metrics.pivot_metrics(self.metrics_rows).to_csv(
            os.path.join(self.out_dir, "metrics.csv"), index=False
        )

    def _add_metric(
        self, category: str, experiment: str, name: str, type_: str, value
    ) -> None:
        """The one path into self.metrics_rows, so metrics.csv and the
        scalars tab are always the complete scalar record. name is a human
        phrase, no underscores, "ATE" rather than "ate_rmse_m"; type_ is the
        statistic kind, "rmse"/"mean"/"std"/"percentile"/"rate"/"value", so
        two rows can share a name and differ only by type."""
        self.metrics_rows.append(
            {
                "category": category,
                "experiment": experiment,
                "name": name,
                "type": type_,
                "value": value,
            }
        )

    def _wh(self, cfg_entry: dict) -> dict:
        """Extracts an optional per-plot width/height from a config.json
        entry, to splat into the matching visualisation.py call."""
        return {k: cfg_entry[k] for k in ("width", "height") if k in cfg_entry}

    def _analyse_trajectory(self, cfg):
        figs = []
        gt_eval = self.reader.load("gt_eval")
        vio_init = self.reader.load("vio_init")
        ta = TrajectoryAnalysis(cfg.get("alignment_mode", "posyaw"))
        t_ns_per_run, est_xyz_per_run, err_per_run, gt_xyz_per_run = {}, {}, {}, {}
        err_xyz_per_run = {}
        rpe_rmse_series, rpe_length_series = {}, {}

        for exp, df in gt_eval.items():
            df = df.dropna(subset=["p_gt_0", "p_w_i_0"]).reset_index(drop=True)
            if df.empty:
                continue
            t_ns = df["t_ns"].to_numpy()
            gt_xyz, gt_q = self.reader.vec(df, "p_gt", 3), self.reader.vec(
                df, "q_gt", 4
            )
            est_xyz, est_q = self.reader.vec(df, "p_w_i", 3), self.reader.vec(
                df, "q_w_i", 4
            )
            ta.set_alignment(gt_xyz, gt_q, est_xyz, est_q)
            est_xyz_aligned, est_q_aligned = ta.apply(est_xyz, est_q)
            t_ns_per_run[exp] = t_ns
            est_xyz_per_run[exp] = est_xyz_aligned
            gt_xyz_per_run[exp] = gt_xyz

            if cfg.get("ate", {}).get("enabled"):
                ate = ta.ate(gt_xyz, est_xyz_aligned)
                for stat in ("rmse", "mean", "median", "std", "max"):
                    self._add_metric("trajectory", exp, "ATE (m)", stat, ate[stat])
                err_per_run[exp] = ate["residual"]

            if cfg.get("trajectory_error_xyz", {}).get("enabled"):
                err_xyz_per_run[exp] = est_xyz_aligned - gt_xyz

            if cfg.get("rpe_rmse", {}).get("enabled"):
                curve, per_delta_series = ta.rpe(
                    gt_xyz,
                    gt_q,
                    est_xyz_aligned,
                    est_q_aligned,
                    cfg["rpe_rmse"]["deltas"],
                )
                for delta, rmse in curve:
                    self._add_metric(
                        "trajectory", exp, f"RPE RMSE, {delta} frames (m)", "rmse", rmse
                    )
                rpe_rmse_series[exp] = per_delta_series

            if cfg.get("rpe_length", {}).get("enabled"):
                lengths = cfg["rpe_length"]["length"]
                lengths = lengths if isinstance(lengths, list) else [lengths]
                _, per_delta_series = ta.rpe(
                    gt_xyz, gt_q, est_xyz_aligned, est_q_aligned, lengths
                )
                rpe_length_series[exp] = (t_ns, per_delta_series)

            if cfg.get("yaw_offset", {}).get("enabled") and exp in vio_init:
                self._add_metric(
                    "trajectory",
                    exp,
                    "Yaw offset (rad)",
                    "value",
                    vio_init[exp]["yaw_offset_rad"].iloc[0],
                )

            if cfg.get("drift_proxies", {}).get("enabled"):
                proxies = metrics.drift_proxies(gt_xyz, gt_q, est_xyz, est_q)
                for key, value in proxies.items():
                    name, type_ = DRIFT_PROXY_LABELS[key]
                    self._add_metric("trajectory", exp, name, type_, value)
            ta.reset_alignment()

        if cfg.get("trajectory_3d", {}).get("enabled"):
            colour = (
                err_per_run
                if cfg["trajectory_3d"].get("color_by_error") and err_per_run
                else None
            )
            figs.append(
                (
                    "trajectory_3d",
                    visualisation.line_3d(
                        est_xyz_per_run,
                        "Aligned estimated trajectory",
                        colour,
                        **self._wh(cfg["trajectory_3d"]),
                    ),
                )
            )

        combined_gt_t = combined_gt_xyz = None
        if gt_xyz_per_run:
            combined_gt_t, combined_gt_xyz = metrics.combine_ground_truth(
                t_ns_per_run, gt_xyz_per_run
            )

        if (
            cfg.get("trajectory_3d_gt_overlay", {}).get("enabled")
            and combined_gt_xyz is not None
        ):
            figs.append(
                (
                    "trajectory_3d_gt_overlay",
                    visualisation.trajectory_3d_overlay(
                        combined_gt_xyz,
                        est_xyz_per_run,
                        "Ground truth and estimated trajectories",
                        **self._wh(cfg["trajectory_3d_gt_overlay"]),
                    ),
                )
            )

        if cfg.get("trajectory_2d", {}).get("enabled"):
            overlay = cfg["trajectory_2d"].get("axes_overlayed", False)
            axes = ["xyz"] if overlay else list("xyz")
            for axis_group in axes:
                if overlay:
                    y = {
                        f"{exp}, {name}": est_xyz_per_run[exp][:, i]
                        for exp in est_xyz_per_run
                        for i, name in enumerate("xyz")
                    }
                    t = {
                        f"{exp}, {name}": t_ns_per_run[exp]
                        for exp in t_ns_per_run
                        for name in "xyz"
                    }
                    if combined_gt_xyz is not None:
                        for i, name in enumerate("xyz"):
                            y[f"ground truth, {name}"] = combined_gt_xyz[:, i]
                            t[f"ground truth, {name}"] = combined_gt_t
                    figs.append(
                        (
                            "trajectory_2d",
                            visualisation.time_series(
                                t,
                                y,
                                "Estimated position, all axes",
                                "t_ns",
                                "position (m)",
                                link_zoom=True,
                                **self._wh(cfg["trajectory_2d"]),
                            ),
                        )
                    )
                else:
                    i = "xyz".index(axis_group)
                    y = {exp: est_xyz_per_run[exp][:, i] for exp in est_xyz_per_run}
                    t = dict(t_ns_per_run)
                    if combined_gt_xyz is not None:
                        y["ground truth"] = combined_gt_xyz[:, i]
                        t["ground truth"] = combined_gt_t
                    figs.append(
                        (
                            f"trajectory_2d_{axis_group}",
                            visualisation.time_series(
                                t,
                                y,
                                f"Estimated position, {axis_group}",
                                "t_ns",
                                f"{axis_group} (m)",
                                link_zoom=True,
                                **self._wh(cfg["trajectory_2d"]),
                            ),
                        )
                    )

        if cfg.get("trajectory_error", {}).get("enabled") and err_per_run:
            figs.append(
                (
                    "trajectory_error",
                    visualisation.time_series(
                        t_ns_per_run,
                        err_per_run,
                        "Aligned ATE residual over time",
                        "t_ns",
                        "error (m)",
                        link_zoom=True,
                        **self._wh(cfg["trajectory_error"]),
                    ),
                )
            )

        if cfg.get("trajectory_error_xyz", {}).get("enabled") and err_xyz_per_run:
            axes = ["x", "y", "z"]
            quantities = {a: {} for a in axes}
            for exp, residual_xyz in err_xyz_per_run.items():
                for i, a in enumerate(axes):
                    quantities[a][exp] = residual_xyz[:, i]
            figs.append(
                (
                    "trajectory_error_xyz",
                    visualisation.multi_axis_time_series(
                        t_ns_per_run,
                        quantities,
                        "Aligned ATE residual over time, by axis",
                        "t_ns",
                        "error (m)",
                        link_zoom=True,
                        **self._wh(cfg["trajectory_error_xyz"]),
                    ),
                )
            )

        if cfg.get("rpe_rmse", {}).get("enabled") and rpe_rmse_series:
            x_per_run, mean_per_run, std_per_run = {}, {}, {}
            for exp, per_delta_series in rpe_rmse_series.items():
                x_per_run[exp], mean_per_run[exp], std_per_run[exp] = (
                    metrics.rpe_curve_with_band(per_delta_series)
                )
            figs.append(
                (
                    "rpe_rmse_vs_length",
                    visualisation.line_with_band(
                        x_per_run,
                        mean_per_run,
                        std_per_run,
                        "RPE RMSE vs sub trajectory length",
                        "sub trajectory length (frames)",
                        "RPE (m)",
                        **self._wh(cfg["rpe_rmse"]),
                    ),
                )
            )

        if cfg.get("rpe_length", {}).get("enabled") and rpe_length_series:
            lengths = cfg["rpe_length"]["length"]
            lengths = lengths if isinstance(lengths, list) else [lengths]
            for length in lengths:
                t_axis, y = {}, {}
                for exp, (t_ns, per_delta_series) in rpe_length_series.items():
                    if length in per_delta_series:
                        t_axis[exp] = t_ns[:-length]
                        y[exp] = per_delta_series[length]
                if y:
                    figs.append(
                        (
                            f"rpe_per_start_time_{length}",
                            visualisation.time_series(
                                t_axis,
                                y,
                                f"RPE drift rate over time, length {length}",
                                "t_ns",
                                "RPE (m)",
                                link_zoom=True,
                                **self._wh(cfg["rpe_length"]),
                            ),
                        )
                    )
        return figs

    def _analyse_optical_flow_detection(self, cfg):
        figs = []
        if cfg.get("detection_count", {}).get("enabled"):
            of_frame = self.reader.load("of_frame")
            t_ns = {exp: df["t_ns"] for exp, df in of_frame.items()}
            y = {exp: df["detected"] for exp, df in of_frame.items()}
            figs.append(
                (
                    "detection_count",
                    visualisation.time_series(
                        t_ns,
                        y,
                        "Optical flow detections per frame",
                        "t_ns",
                        "detected (count)",
                        link_zoom=True,
                        **self._wh(cfg["detection_count"]),
                    ),
                )
            )
            for exp, df in of_frame.items():
                mean, std = metrics.mean_std_scalars(df, ["detected"])["detected"]
                self._add_metric(
                    "optical_flow_detection", exp, "Detections per frame", "mean", mean
                )
                self._add_metric(
                    "optical_flow_detection", exp, "Detections per frame", "std", std
                )
        return figs

    def _analyse_optical_flow_tracking(self, cfg):
        figs = []
        of_track_death = self.reader.load("of_track_death")

        if cfg.get("track_length_distribution", {}).get("enabled"):
            bins = cfg["track_length_distribution"].get("bins", 30)
            values = {exp: df["age_frames"] for exp, df in of_track_death.items()}
            figs.append(
                (
                    "track_length_distribution",
                    visualisation.histogram(
                        values,
                        "Track length at death",
                        "age (frames)",
                        bins,
                        **self._wh(cfg["track_length_distribution"]),
                    ),
                )
            )
            for exp, df in of_track_death.items():
                mean, std = metrics.mean_std_scalars(df, ["age_frames"])["age_frames"]
                self._add_metric(
                    "optical_flow_tracking",
                    exp,
                    "Track age at death (frames)",
                    "mean",
                    mean,
                )
                self._add_metric(
                    "optical_flow_tracking",
                    exp,
                    "Track age at death (frames)",
                    "std",
                    std,
                )

        if cfg.get("track_survival_curve", {}).get("enabled"):
            x_per_run, y_per_run = {}, {}
            for exp, df in of_track_death.items():
                x_per_run[exp], y_per_run[exp] = metrics.survival_curve(
                    df["age_frames"].to_numpy()
                )
            figs.append(
                (
                    "track_survival_curve",
                    visualisation.line(
                        x_per_run,
                        y_per_run,
                        "Track survival curve",
                        "age (frames)",
                        "fraction still alive",
                        **self._wh(cfg["track_survival_curve"]),
                    ),
                )
            )

        if cfg.get("death_reason_breakdown", {}).get("enabled"):
            counts = {
                exp: metrics.value_counts_dict(df["reason"])
                for exp, df in of_track_death.items()
            }
            figs.append(
                (
                    "death_reason_breakdown",
                    visualisation.stacked_bar(
                        counts,
                        "Track death reason breakdown",
                        "reason, 0 forward fail, 1 backward fail, 2 recovery rejected",
                        **self._wh(cfg["death_reason_breakdown"]),
                    ),
                )
            )

        reproj = self.reader.load("vio_reproj_pre_opt")
        if cfg.get("reprojection_error_distribution", {}).get("enabled"):
            rc = cfg["reprojection_error_distribution"]
            types = rc.get("type", "histogram")
            types = types if isinstance(types, list) else [types]
            if "histogram" in types:
                values = {exp: df["mean_px"] for exp, df in reproj.items()}
                figs.append(
                    (
                        "reprojection_error_histogram",
                        visualisation.histogram(
                            values,
                            "Pre optimisation reprojection error",
                            "mean error (px)",
                            **self._wh(rc),
                        ),
                    )
                )
            if "time_series" in types:
                t_ns = {exp: df["t_ns"] for exp, df in reproj.items()}
                y = {exp: df["p95_px"] for exp, df in reproj.items()}
                figs.append(
                    (
                        "reprojection_error_time_series",
                        visualisation.time_series(
                            t_ns,
                            y,
                            "Pre optimisation reprojection error, p95",
                            "t_ns",
                            "p95 error (px)",
                            link_zoom=True,
                            **self._wh(rc),
                        ),
                    )
                )
            for exp, df in reproj.items():
                mean, std = metrics.mean_std_scalars(df, ["mean_px"])["mean_px"]
                self._add_metric(
                    "optical_flow_tracking",
                    exp,
                    "Reprojection error (px)",
                    "mean",
                    mean,
                )
                self._add_metric(
                    "optical_flow_tracking", exp, "Reprojection error (px)", "std", std
                )

        of_frame = self.reader.load("of_frame")
        if cfg.get("inlier_outlier_ratio", {}).get("enabled"):
            t_ns, y = {}, {}
            for exp, df in of_frame.items():
                t_ns[exp] = df["t_ns"]
                ratio = metrics.inlier_outlier_ratio(df["tracked"], df["epi_rej"])
                y[exp] = ratio
                self._add_metric(
                    "optical_flow_tracking",
                    exp,
                    "Inlier to outlier ratio",
                    "mean",
                    float(np.nanmean(ratio)),
                )
            figs.append(
                (
                    "inlier_outlier_ratio",
                    visualisation.time_series(
                        t_ns,
                        y,
                        "Inlier to outlier ratio",
                        "t_ns",
                        "tracked / (tracked + epi_rej)",
                        link_zoom=True,
                        **self._wh(cfg["inlier_outlier_ratio"]),
                    ),
                )
            )

        if cfg.get("optical_flow_endpoint_error", {}).get("enabled"):
            oc = cfg["optical_flow_endpoint_error"]
            t_ns = {exp: df["t_ns"] for exp, df in of_frame.items()}
            y = {exp: df["recov_rej"] for exp, df in of_frame.items()}
            figs.append(
                (
                    "optical_flow_endpoint_error_time_series",
                    visualisation.time_series(
                        t_ns,
                        y,
                        "Forward backward recovery rejections",
                        "t_ns",
                        "recov_rej (count)",
                        link_zoom=True,
                        **self._wh(oc),
                    ),
                )
            )
            flow = {exp: df["flow_px_mean"] for exp, df in of_frame.items()}
            figs.append(
                (
                    "optical_flow_endpoint_error_histogram",
                    visualisation.histogram(
                        flow,
                        "Interframe flow magnitude",
                        "flow (px per frame)",
                        **self._wh(oc),
                    ),
                )
            )
        return figs

    def _analyse_vio_triangulation(self, cfg):
        figs = []
        triang = self.reader.load("vio_triang")

        if cfg.get("parallax_angle", {}).get("enabled"):
            pc = cfg["parallax_angle"]
            unit_deg = pc.get("unit") == "deg"
            values, t_ns, y = {}, {}, {}
            for exp, df in triang.items():
                angle = (
                    np.degrees(np.arccos(df["cos_parallax_mean"].clip(-1, 1)))
                    if unit_deg
                    else df["cos_parallax_mean"]
                )
                values[exp], t_ns[exp], y[exp] = angle, df["t_ns"], angle
                self._add_metric(
                    "vio_triangulation",
                    exp,
                    "Parallax angle",
                    "mean",
                    float(angle.mean()),
                )
                self._add_metric(
                    "vio_triangulation",
                    exp,
                    "Parallax angle",
                    "std",
                    float(angle.std()),
                )
            unit = "deg" if unit_deg else "cos(angle)"
            figs.append(
                (
                    "parallax_angle_histogram",
                    visualisation.histogram(
                        values, "Parallax angle at triangulation", unit, **self._wh(pc)
                    ),
                )
            )
            figs.append(
                (
                    "parallax_angle_time_series",
                    visualisation.time_series(
                        t_ns,
                        y,
                        "Parallax angle over time",
                        "t_ns",
                        unit,
                        link_zoom=True,
                        **self._wh(pc),
                    ),
                )
            )

        if cfg.get("baseline_distribution", {}).get("enabled"):
            values = {exp: df["baseline_m_max"] for exp, df in triang.items()}
            figs.append(
                (
                    "baseline_distribution",
                    visualisation.histogram(
                        values,
                        "Triangulation baseline",
                        "baseline (m)",
                        **self._wh(cfg["baseline_distribution"]),
                    ),
                )
            )

        if cfg.get("triangulation_funnel", {}).get("enabled"):
            reasons = [
                "no_prior_obs",
                "unproject_fail",
                "short_baseline",
                "not_finite",
                "behind",
                "too_close",
                "low_parallax",
            ]
            counts = {
                exp: {r: int(df[r].sum()) for r in reasons}
                for exp, df in triang.items()
            }
            figs.append(
                (
                    "triangulation_funnel",
                    visualisation.stacked_bar(
                        counts,
                        "VIO triangulation rejection funnel",
                        "rejection reason",
                        **self._wh(cfg["triangulation_funnel"]),
                    ),
                )
            )
        return figs

    def _analyse_vio_optimisation(self, cfg):
        figs = []
        vio_linearise = self.reader.load("vio_linearise")
        vio_solver_iter = self.reader.load("vio_solver_iter")
        vio_solver_summary = self.reader.load("vio_solver_summary")
        vio_marg = self.reader.load("vio_marg")
        vio_marg_nullspace = self.reader.load("vio_marg_nullspace")
        vio_pose_update = self.reader.load("vio_pose_update")
        vio_assoc = self.reader.load("vio_assoc")
        vio_landmarks = self.reader.load("vio_landmarks")
        vio_init = self.reader.load("vio_init")
        vio_state = self.reader.load("vio_state")
        vio_imu_frame = self.reader.load("vio_imu_frame")
        mapper_linearise = self.reader.load("mapper_linearise")
        mapper_solver_iter = self.reader.load("mapper_solver_iter")

        if cfg.get("cost_curve", {}).get("enabled"):
            # Global iteration index across the run rather than per call
            # time, since one line per optimize() call would need thousands
            # of legend entries on a multi-minute flight. Not a shared time
            # base with the rest of the dashboard, so not zoom linked.
            x = {exp: np.arange(len(df)) for exp, df in vio_linearise.items()}
            y = {exp: df["error_total"] for exp, df in vio_linearise.items()}
            figs.append(
                (
                    "cost_curve",
                    visualisation.time_series(
                        x,
                        y,
                        "VIO cost convergence",
                        "global iteration index",
                        "error_total",
                        **self._wh(cfg["cost_curve"]),
                    ),
                )
            )

        if cfg.get("damping_trace", {}).get("enabled"):
            x, y, marker = {}, {}, {}
            for exp, df in vio_solver_iter.items():
                x[exp] = np.arange(len(df))
                y[exp] = df["lambda"]
                marker[exp] = np.flatnonzero((df["status"] == 1).to_numpy())
            figs.append(
                (
                    "damping_trace",
                    visualisation.time_series(
                        x,
                        y,
                        "VIO damping trace, markers are rejected steps",
                        "global iteration index",
                        "lambda",
                        log_y=True,
                        marker_index_per_run=marker,
                        **self._wh(cfg["damping_trace"]),
                    ),
                )
            )

        if cfg.get("accept_reject_ratio", {}).get("enabled"):
            for exp, df in vio_solver_iter.items():
                self._add_metric(
                    "vio_optimisation",
                    exp,
                    "VIO solver accept rate",
                    "rate",
                    metrics.accept_reject_ratio(df["status"]),
                )

        if cfg.get("cost_split", {}).get("enabled"):
            terms = ("vision", "imu", "bias_g", "bias_a", "marg_prior")
            x, quantities = {}, {t: {} for t in terms}
            for exp, df in vio_solver_iter.items():
                x[exp] = np.arange(len(df))
                for t in terms:
                    quantities[t][exp] = df[t]
            figs.append(
                (
                    "cost_split",
                    visualisation.multi_axis_time_series(
                        x,
                        quantities,
                        "VIO cost split by term",
                        "global iteration index",
                        "cost",
                        **self._wh(cfg["cost_split"]),
                    ),
                )
            )

        if cfg.get("solve_outcome", {}).get("enabled"):
            t_ns = {exp: df["t_ns"] for exp, df in vio_solver_summary.items()}
            reason = {exp: df["reason"] for exp, df in vio_solver_summary.items()}
            figs.append(
                (
                    "solve_outcome",
                    visualisation.categorical_timeline(
                        t_ns,
                        reason,
                        "VIO solve outcome, 0 converged, 2 not converged, 3 terminated",
                        "t_ns",
                        link_zoom=True,
                        **self._wh(cfg["solve_outcome"]),
                    ),
                )
            )

        if cfg.get("marg_size", {}).get("enabled"):
            t_ns = {exp: df["t_ns"] for exp, df in vio_marg.items()}
            y = {exp: df["keeping"] for exp, df in vio_marg.items()}
            figs.append(
                (
                    "marg_size",
                    visualisation.time_series(
                        t_ns,
                        y,
                        "Marginalisation window size kept",
                        "t_ns",
                        "states kept (count)",
                        link_zoom=True,
                        **self._wh(cfg["marg_size"]),
                    ),
                )
            )

        if cfg.get("marg_eigenspectrum", {}).get("enabled"):
            t_ns = {exp: df["t_ns"] for exp, df in vio_marg_nullspace.items()}
            for col, label in (
                ("ev_min", "smallest eigenvalue"),
                ("ev_max", "largest eigenvalue"),
                ("ev_condition", "condition number"),
            ):
                y = {exp: df[col] for exp, df in vio_marg_nullspace.items()}
                figs.append(
                    (
                        f"marg_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"Marginalisation prior, {label}",
                            "t_ns",
                            col,
                            log_y=True,
                            link_zoom=True,
                            **self._wh(cfg["marg_eigenspectrum"]),
                        ),
                    )
                )

        if cfg.get("marg_nullspace_response", {}).get("enabled"):
            directions = [
                "trans_x",
                "trans_y",
                "trans_z",
                "roll",
                "pitch",
                "yaw",
                "random_control",
            ]
            t_ns, quantities = {}, {d: {} for d in directions}
            for exp, df in vio_marg_nullspace.items():
                response = metrics.nullspace_response(self.reader.vec(df, "marg_ns", 7))
                t_ns[exp] = df["t_ns"]
                for d in directions:
                    quantities[d][exp] = response[d]
            figs.append(
                (
                    "marg_nullspace_response",
                    visualisation.multi_axis_time_series(
                        t_ns,
                        quantities,
                        "Marginalisation nullspace response, named directions",
                        "t_ns",
                        "xHx + xb",
                        link_zoom=True,
                        **self._wh(cfg["marg_nullspace_response"]),
                    ),
                )
            )

        if cfg.get("pose_update_acceptance", {}).get("enabled"):
            t_ns = {exp: df["t_ns"] for exp, df in vio_pose_update.items()}
            for col in ("applied", "rejected"):
                y = {exp: df[col] for exp, df in vio_pose_update.items()}
                figs.append(
                    (
                        f"pose_update_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"Local mapper pose update, {col}",
                            "t_ns",
                            "count",
                            link_zoom=True,
                            **self._wh(cfg["pose_update_acceptance"]),
                        ),
                    )
                )

        if cfg.get("association_ratio", {}).get("enabled"):
            t_ns, y, marker = {}, {}, {}
            for exp, df in vio_assoc.items():
                t_ns[exp] = df["t_ns"]
                y[exp] = df["connected_ratio"]
                marker[exp] = np.flatnonzero((df["take_kf"] == 1).to_numpy())
            figs.append(
                (
                    "association_ratio",
                    visualisation.time_series(
                        t_ns,
                        y,
                        "Association ratio, markers are keyframes",
                        "t_ns",
                        "connected_ratio",
                        marker_index_per_run=marker,
                        link_zoom=True,
                        **self._wh(cfg["association_ratio"]),
                    ),
                )
            )

        if cfg.get("landmark_churn", {}).get("enabled"):
            t_ns = {exp: df["t_ns"] for exp, df in vio_landmarks.items()}
            for col in ("lost", "lmdb"):
                y = {exp: df[col] for exp, df in vio_landmarks.items()}
                figs.append(
                    (
                        f"landmark_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"VIO landmark database, {col}",
                            "t_ns",
                            "count",
                            link_zoom=True,
                            **self._wh(cfg["landmark_churn"]),
                        ),
                    )
                )

        if cfg.get("init_sanity", {}).get("enabled"):
            for col in ("tilt_deg", "accel_norm", "gyro_norm"):
                for exp, df in vio_init.items():
                    self._add_metric(
                        "vio_optimisation",
                        exp,
                        INIT_SANITY_LABELS[col],
                        "value",
                        df[col].iloc[0],
                    )

        if cfg.get("state_trace", {}).get("enabled"):
            t_ns = {exp: df["t_ns"] for exp, df in vio_state.items()}
            for col, unit in (
                ("p_norm", "m"),
                ("v_norm", "m/s"),
                ("bg_norm", "rad/s"),
                ("ba_norm", "m/s^2"),
            ):
                y = {exp: df[col] for exp, df in vio_state.items()}
                figs.append(
                    (
                        f"state_trace_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"State trace, {col}",
                            "t_ns",
                            f"{col} ({unit})",
                            link_zoom=True,
                            **self._wh(cfg["state_trace"]),
                        ),
                    )
                )

        if cfg.get("excitation_covariance", {}).get("enabled"):
            window_s = cfg["excitation_covariance"].get("window_s", 1.5)
            t_ns, y = {}, {}
            for exp, imu_df in vio_imu_frame.items():
                if exp not in vio_state or exp not in vio_init:
                    continue
                st_df = vio_state[exp]
                g = self.reader.vec(vio_init[exp].iloc[[0]], "g", 3)[0]
                # Joined on t_ns rather than truncated to min(len(imu_df),
                # len(st_df)) and zipped by row position, because the IMU
                # frame record and the state trace are two independent
                # logging sites and a frame dropped by only one of them
                # shifts their row counts out of step, which a positional
                # zip would silently pair against the wrong frame.
                merged = metrics.align_frames_on_t_ns(
                    imu_df[["t_ns", "accel_mean_0", "accel_mean_1", "accel_mean_2"]],
                    st_df[["t_ns", "q_w_i_0", "q_w_i_1", "q_w_i_2", "q_w_i_3"]],
                )
                if merged.empty:
                    continue
                accel_mean_body = self.reader.vec(merged, "accel_mean", 3)
                q = self.reader.vec(merged, "q_w_i", 4)
                R = Rotation.from_quat(q).as_matrix()
                accel_mean_world = np.einsum("nij,nj->ni", R, accel_mean_body) - g
                dt_s = float(np.median(np.diff(merged["t_ns"].to_numpy()))) * 1e-9
                cov_min_ev = metrics.excitation_covariance(
                    accel_mean_world, window_s, dt_s
                )
                t_ns[exp], y[exp] = merged["t_ns"], cov_min_ev
            if y:
                figs.append(
                    (
                        "excitation_covariance",
                        visualisation.time_series(
                            t_ns,
                            y,
                            "Excitation covariance, smallest eigenvalue",
                            "t_ns",
                            "smallest eigenvalue of unit specific force covariance",
                            log_y=True,
                            link_zoom=True,
                            **self._wh(cfg["excitation_covariance"]),
                        ),
                    )
                )

        if cfg.get("stationarity", {}).get("enabled"):
            sc = cfg["stationarity"]
            t_ns = {exp: df["t_ns"] for exp, df in vio_imu_frame.items()}
            flag = {exp: df["is_stationary"] for exp, df in vio_imu_frame.items()}
            figs.append(
                (
                    "stationary_flag",
                    visualisation.categorical_timeline(
                        t_ns,
                        flag,
                        "Zero velocity stationarity test, 1 latched, 0 not",
                        "t_ns",
                        link_zoom=True,
                        **self._wh(sc),
                    ),
                )
            )
            quantities = {
                "specific force deviation from gravity (m/s^2)": {},
                "gyroscope norm (rad/s)": {},
            }
            for exp, df in vio_imu_frame.items():
                quantities["specific force deviation from gravity (m/s^2)"][exp] = (
                    df["accel_deviation_max"]
                )
                quantities["gyroscope norm (rad/s)"][exp] = df["gyro_norm_max"]
            figs.append(
                (
                    "stationarity_deviation",
                    visualisation.multi_axis_time_series(
                        t_ns,
                        quantities,
                        "Stationarity test inputs, maximum deviation per frame",
                        "t_ns",
                        "deviation",
                        link_zoom=True,
                        **self._wh(sc),
                    ),
                )
            )
            for exp, df in vio_imu_frame.items():
                self._add_metric(
                    "vio_optimisation",
                    exp,
                    "Stationary fraction",
                    "rate",
                    float(df["is_stationary"].mean()),
                )

        if cfg.get("mapper_cost_curve", {}).get("enabled"):
            x = {exp: np.arange(len(df)) for exp, df in mapper_linearise.items()}
            y = {exp: df["total"] for exp, df in mapper_linearise.items()}
            figs.append(
                (
                    "mapper_cost_curve",
                    visualisation.time_series(
                        x,
                        y,
                        "Mapper cost convergence, rows already grouped by t_ns and opt_pass at logging time",
                        "global iteration index",
                        "total cost",
                        **self._wh(cfg["mapper_cost_curve"]),
                    ),
                )
            )

        if cfg.get("mapper_damping_accept_reject", {}).get("enabled"):
            x, y = {}, {}
            for exp, df in mapper_solver_iter.items():
                x[exp] = np.arange(len(df))
                y[exp] = df["lambda"]
                self._add_metric(
                    "vio_optimisation",
                    exp,
                    "Mapper solver accept rate",
                    "rate",
                    metrics.accept_reject_ratio(df["status"]),
                )
            figs.append(
                (
                    "mapper_damping_trace",
                    visualisation.time_series(
                        x,
                        y,
                        "Mapper damping trace",
                        "global iteration index",
                        "lambda",
                        log_y=True,
                        **self._wh(cfg["mapper_damping_accept_reject"]),
                    ),
                )
            )
        return figs

    def _analyse_mapping_detection(self, cfg):
        figs = []
        mapper_detect = self.reader.load("mapper_detect")
        if cfg.get("mapper_detection_yield", {}).get("enabled"):
            t_ns = {exp: df["t_ns"] for exp, df in mapper_detect.items()}
            y = {exp: df["corners_total"] for exp, df in mapper_detect.items()}
            figs.append(
                (
                    "mapper_detection_yield",
                    visualisation.time_series(
                        t_ns,
                        y,
                        "Mapper detector, corners_total",
                        "t_ns",
                        "count (count)",
                        link_zoom=True,
                        **self._wh(cfg["mapper_detection_yield"]),
                    ),
                )
            )

        map_tracks = self.reader.load("map_tracks")
        if cfg.get("mapper_track_length_histogram", {}).get("enabled"):
            bin_labels = [str(i + 2) for i in range(9)] + ["11+"]
            counts = {
                exp: self.reader.vec(df, "len_hist", 10).sum(axis=0).tolist()
                for exp, df in map_tracks.items()
            }
            figs.append(
                (
                    "mapper_track_length_histogram",
                    visualisation.histogram_prebinned(
                        bin_labels,
                        counts,
                        "Mapper track length histogram",
                        "track length (frames)",
                        **self._wh(cfg["mapper_track_length_histogram"]),
                    ),
                )
            )

        if cfg.get("map_building_throughput", {}).get("enabled"):
            t_ns = {exp: df["t_ns"] for exp, df in map_tracks.items()}
            for col in ("exported", "new_kfs", "corners"):
                y = {exp: df[col] for exp, df in map_tracks.items()}
                figs.append(
                    (
                        f"map_building_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"Map building throughput, {col}",
                            "t_ns",
                            "count",
                            link_zoom=True,
                            **self._wh(cfg["map_building_throughput"]),
                        ),
                    )
                )
        return figs

    def _analyse_mapping_matching(self, cfg):
        figs = []
        bow_candidate = self.reader.load("map_bow_candidate")
        gt_eval = self.reader.load("gt_eval")
        labels = {}
        if cfg.get("bow_pr_curve", {}).get("enabled"):
            pr_cfg = cfg["bow_pr_curve"]
            x_per_run, y_per_run = {}, {}
            for exp, df in bow_candidate.items():
                if exp not in gt_eval:
                    continue
                recall, precision, is_true = metrics.bow_pr_curve(
                    df,
                    gt_eval[exp],
                    pr_cfg["distance_thresh_m"],
                    pr_cfg["min_time_gap_s"],
                )
                x_per_run[exp], y_per_run[exp], labels[exp] = recall, precision, is_true
            figs.append(
                (
                    "bow_pr_curve",
                    visualisation.line(
                        x_per_run,
                        y_per_run,
                        "BoW retrieval precision recall",
                        "recall",
                        "precision",
                        **self._wh(pr_cfg),
                    ),
                )
            )

        if cfg.get("bow_score_histogram", {}).get("enabled") and labels:
            true_scores = {
                exp: bow_candidate[exp]["score"][labels[exp]] for exp in labels
            }
            false_scores = {
                exp: bow_candidate[exp]["score"][~labels[exp]] for exp in labels
            }
            figs.append(
                (
                    "bow_score_histogram",
                    visualisation.multi_axis_histogram(
                        {"true match": true_scores, "false match": false_scores},
                        "BoW score, true against false match",
                        "score",
                        **self._wh(cfg["bow_score_histogram"]),
                    ),
                )
            )

        bow_query = self.reader.load("map_bow_query")
        if cfg.get("bow_raw_retrieval_yield", {}).get("enabled"):
            bc = cfg["bow_raw_retrieval_yield"]
            t_ns = {exp: df["t_ns"] for exp, df in bow_query.items()}
            for col in ("db_hits", "above_thresh", "best_score"):
                y = {exp: df[col] for exp, df in bow_query.items()}
                figs.append(
                    (
                        f"bow_query_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"BoW query, {col}",
                            "t_ns",
                            col,
                            link_zoom=True,
                            **self._wh(bc),
                        ),
                    )
                )

        pair_match = self.reader.load("map_pair_match")
        if cfg.get("pairwise_match_funnel", {}).get("enabled"):
            counts = {
                exp: {
                    "descriptor_matches": int(df["descriptor_matches"].sum()),
                    "ransac_inliers": int(df["ransac_inliers"].sum()),
                    "stored": int(df["stored"].sum()),
                }
                for exp, df in pair_match.items()
            }
            figs.append(
                (
                    "pairwise_match_funnel",
                    visualisation.stacked_bar(
                        counts,
                        "Pairwise match verification funnel",
                        "stage",
                        **self._wh(cfg["pairwise_match_funnel"]),
                    ),
                )
            )

        match_summary = self.reader.load("map_match_summary")
        if cfg.get("matching_stage_summary", {}).get("enabled"):
            mc = cfg["matching_stage_summary"]
            t_ns = {exp: df["t_ns"] for exp, df in match_summary.items()}
            for col in ("pairs", "inlier_matches", "total_matches"):
                y = {exp: df[col] for exp, df in match_summary.items()}
                figs.append(
                    (
                        f"matching_stage_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"Matching stage summary, {col}",
                            "t_ns",
                            "count",
                            link_zoom=True,
                            **self._wh(mc),
                        ),
                    )
                )
        return figs

    def _analyse_mapping_triangulation(self, cfg):
        figs = []
        if cfg.get("mapper_triangulation_funnel", {}).get("enabled"):
            setup_opt = self.reader.load("map_setup_opt")
            reasons = [
                "short_track",
                "known",
                "retired",
                "no_triang",
                "no_corners",
                "no_host_pose",
                "no_obs_pose",
                "short_baseline",
                "low_parallax",
                "bad_depth",
                "behind",
                "too_close",
                "nan",
            ]
            counts = {
                exp: {r: int(df[r].sum()) for r in reasons}
                for exp, df in setup_opt.items()
            }
            figs.append(
                (
                    "mapper_triangulation_funnel",
                    visualisation.stacked_bar(
                        counts,
                        "Mapper triangulation rejection funnel",
                        "rejection reason",
                        **self._wh(cfg["mapper_triangulation_funnel"]),
                    ),
                )
            )
        return figs

    def _analyse_mapping_optimisation(self, cfg):
        figs = []
        if cfg.get("covisibility_graph", {}).get("enabled"):
            covis = self.reader.load("map_covis")
            # map_covis.csv logs cell/frame counts rather than a full
            # adjacency matrix, so this is an adjacency proxy, not a graph
            # layout.
            matrices = {
                exp: df[["frames", "ref_frames"]].to_numpy()
                for exp, df in covis.items()
            }
            if matrices:
                figs.append(
                    (
                        "covisibility_graph",
                        visualisation.heatmap(
                            matrices,
                            "Covisibility graph proxy, frames against ref_frames",
                            **self._wh(cfg["covisibility_graph"]),
                        ),
                    )
                )

        cull = self.reader.load("map_cull")
        cull_select = self.reader.load("map_cull_select")
        if cfg.get("culling_redundancy_calibration", {}).get("enabled"):
            bin_labels = [str(i) for i in range(9)] + ["9+"]
            counts = {
                exp: self.reader.vec(df, "observers_hist", 10).sum(axis=0).tolist()
                for exp, df in cull_select.items()
            }
            figs.append(
                (
                    "culling_redundancy_calibration",
                    visualisation.histogram_prebinned(
                        bin_labels,
                        counts,
                        "Culling redundancy calibration",
                        "observers per landmark",
                        **self._wh(cfg["culling_redundancy_calibration"]),
                    ),
                )
            )

        if cfg.get("keyframe_culling_rate", {}).get("enabled"):
            for exp, df in cull.items():
                self._add_metric(
                    "mapping_optimisation",
                    exp,
                    "Keyframe culling rate",
                    "rate",
                    metrics.keyframe_culling_rate(df),
                )

        if cfg.get("landmark_filter_attrition", {}).get("enabled"):
            fc = cfg["landmark_filter_attrition"]
            filt = self.reader.load("map_filter")
            t_ns = {exp: df["t_ns"] for exp, df in filt.items()}
            for col in ("lm_before", "lm_after"):
                y = {exp: df[col] for exp, df in filt.items()}
                figs.append(
                    (
                        f"landmark_filter_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"Landmark filter, {col}",
                            "t_ns",
                            "count",
                            link_zoom=True,
                            **self._wh(fc),
                        ),
                    )
                )

        if cfg.get("publish_consistency", {}).get("enabled"):
            pc = cfg["publish_consistency"]
            publish = self.reader.load("map_publish")
            t_ns = {exp: df["t_ns"] for exp, df in publish.items()}
            for col in ("points", "lmdb_landmarks"):
                y = {exp: df[col] for exp, df in publish.items()}
                figs.append(
                    (
                        f"publish_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"Map publish, {col}",
                            "t_ns",
                            "count",
                            link_zoom=True,
                            **self._wh(pc),
                        ),
                    )
                )

        if cfg.get("landmark_rehost_funnel", {}).get("enabled"):
            rehost = self.reader.load("map_rehost")
            counts = {
                exp: metrics.value_counts_dict(
                    df["outcome"].astype(str) + "/" + df["reason"].astype(str)
                )
                for exp, df in rehost.items()
            }
            figs.append(
                (
                    "landmark_rehost_funnel",
                    visualisation.stacked_bar(
                        counts,
                        "Landmark rehost outcome/reason funnel",
                        "outcome/reason",
                        **self._wh(cfg["landmark_rehost_funnel"]),
                    ),
                )
            )

        if cfg.get("culled_keyframe_summary", {}).get("enabled"):
            kc = cfg["culled_keyframe_summary"]
            t_ns = {exp: df["t_ns"] for exp, df in cull.items()}
            for col in ("victims", "rehost_attempted", "no_host"):
                y = {exp: df[col] for exp, df in cull.items()}
                figs.append(
                    (
                        f"culled_keyframe_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"Culled keyframe summary, {col}",
                            "t_ns",
                            "count",
                            link_zoom=True,
                            **self._wh(kc),
                        ),
                    )
                )
        return figs

    def _analyse_timing(self, cfg):
        figs = []
        percentiles = cfg.get("tail_latency_percentiles", {}).get(
            "percentiles", [95, 99]
        )
        want_percentiles = cfg.get("tail_latency_percentiles", {}).get("enabled", False)

        def log_stage_scalars(prefix, exp, df, stages):
            """Mean, standard deviation and (when enabled) tail percentiles
            for every named stage column, one shared code path so every
            timing block below reports the same statistics under the same
            human phrase, STAGE_PREFIX_LABELS/STAGE_LABELS looked up rather
            than the raw snake_case/camelCase column name."""
            prefix_label = STAGE_PREFIX_LABELS[prefix]
            means_stds = metrics.mean_std_scalars(df, stages)
            for stage, (mean, std) in means_stds.items():
                label = f"{prefix_label} {STAGE_LABELS.get(stage, stage)} latency"
                self._add_metric("timing", exp, label, "mean", mean)
                self._add_metric("timing", exp, label, "std", std)
            if want_percentiles:
                tp = metrics.tail_percentiles(df, stages, percentiles)
                for stage in stages:
                    label = f"{prefix_label} {STAGE_LABELS.get(stage, stage)} latency"
                    for p in percentiles:
                        key = f"{stage}_p{int(p)}"
                        if key in tp:
                            self._add_metric(
                                "timing",
                                exp,
                                f"{label}, p{int(p)}",
                                "percentile",
                                tp[key],
                            )

        of_timing = self.reader.load("of_timing")
        if cfg.get("frontend_stage_latency", {}).get("enabled"):
            stages = ["pyramid", "track", "detect_add"]
            t_ns, quantities = {}, {s: {} for s in stages}
            for exp, df in of_timing.items():
                t_ns[exp] = df["t_ns"]
                for s in stages:
                    quantities[s][exp] = df[s]
                log_stage_scalars("of", exp, df, stages + ["total"])
            figs.append(
                (
                    "frontend_stage_latency",
                    visualisation.multi_axis_time_series(
                        t_ns,
                        quantities,
                        "Front end stage latency",
                        "t_ns",
                        "ms",
                        link_zoom=True,
                        **self._wh(cfg["frontend_stage_latency"]),
                    ),
                )
            )

        vio_timing = self.reader.load("vio_timing")
        if cfg.get("vio_stage_latency", {}).get("enabled"):
            stages = [
                "pose_update",
                "association",
                "triangulation",
                "lost_scan",
                "opt_and_marg",
                "publish",
            ]
            t_ns, quantities = {}, {s: {} for s in stages}
            for exp, df in vio_timing.items():
                t_ns[exp] = df["t_ns"]
                for s in stages:
                    quantities[s][exp] = df[s]
                log_stage_scalars("vio", exp, df, stages + ["measure", "process_frame"])
            figs.append(
                (
                    "vio_stage_latency",
                    visualisation.multi_axis_time_series(
                        t_ns,
                        quantities,
                        "VIO stage latency",
                        "t_ns",
                        "ms",
                        link_zoom=True,
                        **self._wh(cfg["vio_stage_latency"]),
                    ),
                )
            )

        if cfg.get("vio_queue_depth", {}).get("enabled"):
            qc = cfg["vio_queue_depth"]
            t_ns = {exp: df["t_ns"] for exp, df in vio_timing.items()}
            for col in ("vision_queue", "imu_queue"):
                y = {exp: df[col] for exp, df in vio_timing.items()}
                figs.append(
                    (
                        f"vio_queue_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"VIO queue depth, {col}",
                            "t_ns",
                            "count",
                            link_zoom=True,
                            **self._wh(qc),
                        ),
                    )
                )

        solver_timing = self.reader.load("vio_solver_timing")
        if cfg.get("vio_solver_stage_latency", {}).get("enabled"):
            stages = [
                "allocateLMB",
                "linearizeProblem",
                "performQR",
                "get_dense_H_b",
                "solve",
                "backSubstitute",
                "computerError2",
            ]
            t_ns, quantities = {}, {s: {} for s in stages}
            for exp, df in solver_timing.items():
                t_ns[exp] = df["t_ns"]
                for s in stages:
                    quantities[s][exp] = df[s]
                log_stage_scalars("vio_solver", exp, df, stages)
            figs.append(
                (
                    "vio_solver_stage_latency",
                    visualisation.multi_axis_time_series(
                        t_ns,
                        quantities,
                        "VIO solver internal stage latency",
                        "t_ns",
                        "ms",
                        link_zoom=True,
                        **self._wh(cfg["vio_solver_stage_latency"]),
                    ),
                )
            )

        marg_timing = self.reader.load("vio_marg_timing")
        if cfg.get("vio_marg_stage_latency", {}).get("enabled"):
            stages = ["marg_linearize", "marg_helper", "marg", "marg_log"]
            t_ns, quantities = {}, {s: {} for s in stages}
            for exp, df in marg_timing.items():
                t_ns[exp] = df["t_ns"]
                for s in stages:
                    quantities[s][exp] = df[s]
                log_stage_scalars("vio_marg", exp, df, stages)
            figs.append(
                (
                    "vio_marg_stage_latency",
                    visualisation.multi_axis_time_series(
                        t_ns,
                        quantities,
                        "VIO marginalisation stage latency",
                        "t_ns",
                        "ms",
                        link_zoom=True,
                        **self._wh(cfg["vio_marg_stage_latency"]),
                    ),
                )
            )

        cycle_timing = self.reader.load("map_cycle_timing")
        if cfg.get("mapper_cycle_latency", {}).get("enabled"):
            stages = [
                "ingest",
                "detect_keypoints",
                "match_stereo",
                "match_local",
                "collect_new_kfs",
                "build_tracks",
                "setup_opt",
                "cull",
                "optimize_1",
                "filter_outliers",
                "optimize_2",
                "publish_vis",
                "pose_update_cb",
            ]
            t_ns, quantities = {}, {s: {} for s in stages}
            for exp, df in cycle_timing.items():
                t_ns[exp] = df["t_ns"]
                for s in stages:
                    quantities[s][exp] = df[s]
                log_stage_scalars("mapper", exp, df, stages + ["total", "queue_wait"])
                self._add_metric(
                    "timing",
                    exp,
                    "Mapper idle fraction",
                    "mean",
                    float(metrics.mapper_idle_fraction(df).mean()),
                )
            figs.append(
                (
                    "mapper_cycle_latency",
                    visualisation.multi_axis_time_series(
                        t_ns,
                        quantities,
                        "Mapper cycle latency",
                        "t_ns",
                        "ms",
                        link_zoom=True,
                        **self._wh(cfg["mapper_cycle_latency"]),
                    ),
                )
            )

        mapper_detect = self.reader.load("mapper_detect")
        if cfg.get("mapper_detection_latency", {}).get("enabled"):
            mdc = cfg["mapper_detection_latency"]
            t_ns = {exp: df["t_ns"] for exp, df in mapper_detect.items()}
            y = {exp: df["detection"] for exp, df in mapper_detect.items()}
            figs.append(
                (
                    "mapper_detection_latency",
                    visualisation.time_series(
                        t_ns,
                        y,
                        "Mapper detector latency",
                        "t_ns",
                        "ms",
                        link_zoom=True,
                        **self._wh(mdc),
                    ),
                )
            )
            for exp, df in mapper_detect.items():
                log_stage_scalars("mapper_detect", exp, df, ["detection"])

        match_summary = self.reader.load("map_match_summary")
        if cfg.get("mapper_matching_latency", {}).get("enabled"):
            stages = ["db_query", "matching"]
            t_ns, quantities = {}, {s: {} for s in stages}
            for exp, df in match_summary.items():
                t_ns[exp] = df["t_ns"]
                for s in stages:
                    quantities[s][exp] = df[s]
                log_stage_scalars("mapper_matching", exp, df, stages)
            figs.append(
                (
                    "mapper_matching_latency",
                    visualisation.multi_axis_time_series(
                        t_ns,
                        quantities,
                        "Mapper matching latency",
                        "t_ns",
                        "ms",
                        link_zoom=True,
                        **self._wh(cfg["mapper_matching_latency"]),
                    ),
                )
            )

        if cfg.get("imu_ingestion_health", {}).get("enabled"):
            ic = cfg["imu_ingestion_health"]
            imu_ingest = self.reader.load("vio_imu_ingest")
            t_ns = {exp: df["t_ns"] for exp, df in imu_ingest.items()}
            for col in ("stamp_rate_hz", "wall_rate_hz", "max_gap", "non_monotonic"):
                y = {exp: df[col] for exp, df in imu_ingest.items()}
                figs.append(
                    (
                        f"imu_ingest_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"IMU ingestion, {col}",
                            "t_ns",
                            col,
                            link_zoom=True,
                            **self._wh(ic),
                        ),
                    )
                )

        if cfg.get("imu_frame_coverage", {}).get("enabled"):
            fc = cfg["imu_frame_coverage"]
            imu_frame = self.reader.load("vio_imu_frame")
            t_ns = {exp: df["t_ns"] for exp, df in imu_frame.items()}
            for col in ("coverage", "stretch_span"):
                y = {exp: df[col] for exp, df in imu_frame.items()}
                figs.append(
                    (
                        f"imu_frame_{col}",
                        visualisation.time_series(
                            t_ns,
                            y,
                            f"IMU frame, {col}",
                            "t_ns",
                            col,
                            link_zoom=True,
                            **self._wh(fc),
                        ),
                    )
                )
        return figs

    def _analyse_cross_run(self, cfg):
        figs = []
        if cfg.get("config_diff", {}).get("enabled"):
            dc = cfg["config_diff"]
            configs = self.reader.load_json("config")
            calibrations = self.reader.load_json("calibration")
            baseline = dc.get("baseline", "first")
            baseline_exp = (
                self.reader.experiment_ids[0] if baseline == "first" else baseline
            )
            if configs:
                figs.append(
                    (
                        "config_diff",
                        visualisation.diff_table(
                            metrics.config_diff(configs, baseline_exp),
                            f"config.json diff against {baseline_exp}",
                            **self._wh(dc),
                        ),
                    )
                )
            if calibrations:
                figs.append(
                    (
                        "calibration_diff",
                        visualisation.diff_table(
                            metrics.config_diff(calibrations, baseline_exp),
                            f"calibration.json diff against {baseline_exp}",
                            **self._wh(dc),
                        ),
                    )
                )

        if cfg.get("distribution_comparison", {}).get("enabled"):
            dc = cfg["distribution_comparison"]
            # Re-read the raw per frame values here since metrics_rows only
            # stores scalars, not a full distribution.
            source = {
                "reprojection_error_distribution": ("vio_reproj_pre_opt", "mean_px"),
                "parallax_angle": ("vio_triang", "cos_parallax_mean"),
                "vio_stage_latency": ("vio_timing", "opt_and_marg"),
            }
            for name in dc.get("metrics", []):
                if name not in source:
                    continue
                csv_name, col = source[name]
                data = self.reader.load(csv_name)
                values_per_run = {
                    exp: df[col].dropna().tolist() for exp, df in data.items()
                }
                figs.append(
                    (
                        f"distribution_{name}",
                        visualisation.box_violin(
                            values_per_run,
                            f"Cross run distribution, {name}",
                            col,
                            **self._wh(dc),
                        ),
                    )
                )
        return figs
```

Every method above follows the same shape section 7's routing table already specifies, load the file or files that tab's rows name, guard each block on that row's own `config.json` key, call the named `metrics.py` function where the row has one, build the named `visualisation.py` figure, and append every scalar through `self._add_metric`. Nothing here is a further design decision, each block is section 7's own row read back as code.

## 10. dashboard.py and the Panel shell

This section described `dashboard.py` as a `.save()`-producing CLI script, `pn.Tabs` holding a `pn.pane.Plotly` per figure, and a post-save `_inject_linked_zoom` script walking the saved page's shadow DOM to wire every figure's `plotly_relayout` event to every other figure tagged `link_zoom`, all of it rendered eagerly into one static `dashboard.html`. **Correction, 2026-10-20.** `dashboard.py` is now a live `panel serve` app, not a script that saves a file, `pn.Tabs(dynamic=True)` lazily attaches a tab's content only when it is first selected, and cross-figure zoom linking, `time_series`/`multi_axis_time_series`/`categorical_timeline` only, is now Bokeh's own shared `Range1d` mechanism rather than `_inject_linked_zoom`'s hand-written `plotly_relayout`/shadow-DOM walk, which no longer exists. `plans/dashboard_live_migration.md` §3/§4.3 carries the current `dashboard.py` design and code in full; the shadow-DOM/`DOMPurify`-collision/`findAll` discussion this section used to carry below the old code listing no longer applies to anything in the current file and is not reproduced there, it was a property of the removed `_inject_linked_zoom` approach specifically, not of Panel/Bokeh generally.

## 11. config.json, full schema

```json
{
  "trajectory": {
    "alignment_mode": "posyaw",
    "ate": {"enabled": true},
    "trajectory_3d": {"enabled": true, "color_by_error": true, "width": 1080, "height": 810},
    "trajectory_3d_gt_overlay": {"enabled": true, "width": 1080, "height": 810},
    "trajectory_2d": {"enabled": true, "axes_overlayed": false, "width": 1080, "height": 400},
    "trajectory_error": {"enabled": true, "axes_overlayed": true, "width": 1080, "height": 400},
    "trajectory_error_xyz": {"enabled": true, "width": 1080, "height": 650},
    "rpe_rmse": {"enabled": true, "deltas": [5, 10, 20, 30, 50, 75, 100, 150, 200, 300],
                 "width": 720, "height": 540},
    "rpe_length": {"enabled": true, "length": [10, 40, 60], "width": 1080, "height": 400},
    "yaw_offset": {"enabled": true},
    "drift_proxies": {"enabled": true},
    "tracking_success": {"enabled": false}
  },
  "optical_flow_detection": {
    "detection_count": {"enabled": true, "width": 1080, "height": 400},
    "occupancy_grid": {"enabled": false}
  },
  "optical_flow_tracking": {
    "track_length_distribution": {"enabled": true, "bins": 30, "width": 720, "height": 540},
    "track_survival_curve": {"enabled": true, "width": 720, "height": 540},
    "death_reason_breakdown": {"enabled": true, "width": 720, "height": 540},
    "reprojection_error_distribution": {"enabled": true, "type": ["histogram", "time_series"],
                                         "width": 1080, "height": 450},
    "inlier_outlier_ratio": {"enabled": true, "width": 1080, "height": 400},
    "optical_flow_endpoint_error": {"enabled": true, "width": 1080, "height": 450}
  },
  "vio_triangulation": {
    "parallax_angle": {"enabled": true, "unit": "deg", "width": 1080, "height": 450},
    "baseline_distribution": {"enabled": true, "width": 720, "height": 540},
    "triangulation_funnel": {"enabled": true, "width": 720, "height": 540}
  },
  "vio_optimisation": {
    "cost_curve": {"enabled": true, "width": 1080, "height": 400},
    "damping_trace": {"enabled": true, "width": 1080, "height": 400},
    "accept_reject_ratio": {"enabled": true},
    "cost_split": {"enabled": true, "width": 1080, "height": 900},
    "solve_outcome": {"enabled": true, "width": 1080, "height": 400},
    "marg_size": {"enabled": true, "width": 1080, "height": 400},
    "marg_eigenspectrum": {"enabled": true, "width": 1080, "height": 400},
    "marg_nullspace_response": {"enabled": true, "width": 1080, "height": 1250},
    "pose_update_acceptance": {"enabled": true, "width": 1080, "height": 400},
    "association_ratio": {"enabled": true, "width": 1080, "height": 400},
    "landmark_churn": {"enabled": true, "width": 1080, "height": 400},
    "init_sanity": {"enabled": true},
    "state_trace": {"enabled": true, "width": 1080, "height": 400},
    "excitation_covariance": {"enabled": true, "window_s": 1.5, "width": 1080, "height": 400},
    "stationarity": {"enabled": true, "width": 1080, "height": 400},
    "mapper_cost_curve": {"enabled": true, "width": 1080, "height": 400},
    "mapper_damping_accept_reject": {"enabled": true, "width": 1080, "height": 400}
  },
  "mapping_detection": {
    "mapper_detection_yield": {"enabled": true, "width": 1080, "height": 400},
    "mapper_track_length_histogram": {"enabled": true, "width": 720, "height": 540},
    "map_building_throughput": {"enabled": true, "width": 1080, "height": 400}
  },
  "mapping_matching": {
    "bow_pr_curve": {"enabled": true, "distance_thresh_m": 5.0, "min_time_gap_s": 2.0,
                      "width": 720, "height": 540},
    "bow_score_histogram": {"enabled": true, "width": 1080, "height": 500},
    "bow_raw_retrieval_yield": {"enabled": true, "width": 1080, "height": 400},
    "pairwise_match_funnel": {"enabled": true, "width": 720, "height": 540},
    "matching_stage_summary": {"enabled": true, "width": 1080, "height": 400}
  },
  "mapping_triangulation": {
    "mapper_triangulation_funnel": {"enabled": true, "width": 720, "height": 540}
  },
  "mapping_optimisation": {
    "covisibility_graph": {"enabled": false, "width": 720, "height": 540},
    "culling_redundancy_calibration": {"enabled": true, "width": 720, "height": 540},
    "keyframe_culling_rate": {"enabled": true},
    "landmark_filter_attrition": {"enabled": true, "width": 1080, "height": 400},
    "publish_consistency": {"enabled": true, "width": 1080, "height": 400},
    "landmark_rehost_funnel": {"enabled": true, "width": 720, "height": 540},
    "culled_keyframe_summary": {"enabled": true, "width": 1080, "height": 400}
  },
  "timing": {
    "frontend_stage_latency": {"enabled": true, "width": 1080, "height": 650},
    "vio_stage_latency": {"enabled": true, "width": 1080, "height": 1100},
    "vio_queue_depth": {"enabled": true, "width": 1080, "height": 400},
    "vio_solver_stage_latency": {"enabled": true, "width": 1080, "height": 1250},
    "vio_marg_stage_latency": {"enabled": true, "width": 1080, "height": 750},
    "mapper_cycle_latency": {"enabled": true, "width": 1080, "height": 2000},
    "mapper_detection_latency": {"enabled": true, "width": 1080, "height": 400},
    "mapper_matching_latency": {"enabled": true, "width": 1080, "height": 400},
    "imu_ingestion_health": {"enabled": true, "width": 1080, "height": 400},
    "imu_frame_coverage": {"enabled": true, "width": 1080, "height": 400},
    "tail_latency_percentiles": {"enabled": true, "percentiles": [95, 99]}
  },
  "cross_run": {
    "config_diff": {"enabled": true, "baseline": "first", "width": 1080, "height": 540},
    "distribution_comparison": {"enabled": true,
      "metrics": ["reprojection_error_distribution", "parallax_angle", "vio_stage_latency"],
      "width": 720, "height": 540}
  }
}
```

Every key with a `type` or `length` field accepting a list follows one convention, read once in `class Analysis`, a scalar produces one figure, a list produces one figure per entry, `reprojection_error_distribution.type` and `rpe_length.length` are both already written as lists above to exercise this path from the first run. `"baseline": "first"` in `config_diff` resolves to `DataReader.experiment_ids[0]`, the lexicographically first, hence chronologically first, timestamped directory, an explicit experiment id may be given instead. `alignment_mode` is a per invocation top level choice rather than an auto detected one, because neither `config.json` nor `calibration.json` the SLAM run itself writes names whether the run used the IMU or how many cameras it fused strongly enough to infer this reliably, `calibration.json`'s own `intrinsics` array length only distinguishes one camera from two, so a wrong guess would silently launder a real scale or attitude error into an alignment artefact exactly as `context/slam_analysis.md` section 2.1 warns against, and an explicit choice the operator makes once per dataset is a smaller and safer surface than a heuristic.

`"width"`/`"height"` are set explicitly on every entry that produces a figure, read by `class Analysis._wh` and passed to the matching `visualisation.py` call, section 8's own 1440px viewport reasoning is what sets each one's actual number. An entry with no figure of its own, a scalar only quantity such as `ate`/`yaw_offset`/`drift_proxies`/`init_sanity`/`keyframe_culling_rate`/`accept_reject_ratio`/`tail_latency_percentiles`, carries neither key, since `_wh` is never called for it and an unread key would be dead configuration surface.

## 12. Deprecation of `scripts/vio_log_extract.py` and `scripts/vio_log_plot.py`

Neither script has any consumer outside its own docstring and the two plan documents that describe how it was used historically, `plans/vio_drift_analysis.md` and `context/vio_optical_flow_instrumentation.md`, both of which describe completed investigative work rather than a standing dependency, and no `djinn` workflow, CI file or `CMakeLists.txt` references either script. `plans/SLAM_logging.md` line 1162 previously recorded a decision to keep both scripts indefinitely, correct as of the change it describes, superseded here because this dashboard is the successor the same paragraph anticipated and its own console text input has no runs left to serve once every capture of interest has been re-run under the CSV writer. That correction is applied directly to `plans/SLAM_logging.md` in Phase 5, alongside the two files' own deletion, not in this plan document, since deleting a file and editing a different plan's own prior claim are both implementation actions the approval gate below covers.

## 13. Dependencies

`pandas`, `plotly`, `scipy` and `panel` are required, none are present in the development container today, confirmed by `python3 -c "import pandas"` failing here, and are installed the same way `matplotlib` already is for `scripts/vio_log_plot.py`, `pip3 install --break-system-packages pandas plotly scipy panel`. `panel` itself pulls in Bokeh, Param and Tornado among its twelve hard dependencies, section 2.3's own accepted cost of the minimal code path, `itables` plus a Bootstrap tab strip is the two dependency fallback recorded there should this weight become a problem in whichever environment eventually runs `dashboard.py`. `cupy` is optional, imported inside a `try`/`except` exactly as `context/slam_analysis.md` section 2.2's own code already does, so `dashboard.py` runs correctly with no GPU and no CuPy installed, falling back to plain NumPy for the RPE sweep at a cost that only matters once a run's `gt_eval.csv` reaches tens of thousands of rows swept across a long `deltas` list. Section 16 gives the concrete placement, `docker/analysis/Dockerfile`, a standalone image built for this script, carries all four packages plus `cupy` by default, so a run inside `analysis_container` gets CuPy acceleration with no manual install, while the `try`/`except` above still covers every other environment that runs this code.

## 14. Validation strategy

| Check | How |
|---|---|
| Every row of section 7's routing table has a config key, a reader call and a function | Cross read against `context/slam_analysis.md` sections 2 through 9, every quantity named there appears exactly once above or in the explicitly deferred section 7 list |
| `DataReader.load` never raises on a missing or empty file | Exercised directly against `/ws/logs/2026-09-26_23-06-59/`, where `mapper_stereo.csv` is present but empty |
| Quaternion order | Confirmed at `src/controller.cpp:255-256`, `qWI = current_latest_pose_->T_w_i.unit_quaternion().coeffs()`, Eigen's `x, y, z, w`, matching `scipy.spatial.transform.Rotation.from_quat`'s own default order with no reordering needed |
| `t_ns` is a shared axis across every file | Confirmed against the sample log, `gt_eval.csv`/`vio_state.csv`/`of_frame.csv` share 3953 rows, `vio_triang.csv`/`map_cycle_timing.csv`/`mapper_detect.csv`/`map_bow_query.csv` share 1297, the keyframe subset of the first axis, so a `t_ns` join between any two files in this stack is either exact or a `merge_asof` against the same underlying frame clock, never a foreign timestamp domain |
| Alignment mode is explicit, not inferred | `config.json`'s own `alignment_mode` key, rationale in section 11 |
| `figures/*.html` open with no internet reachable | Each is written with `include_plotlyjs=True` (Plotly) or `resources=INLINE` (Bokeh), neither path fetches a CDN script. **Correction, 2026-10-20.** This row originally asserted the same of the combined `dashboard.html`; there is no longer such a file, the live dashboard page itself is reached over a live connection by design, `plans/dashboard_live_migration.md` |
| The metrics table filters without a live Python kernel | `pn.widgets.Tabulator`'s `header_filters=True` is documented client side filtering, confirmed against `[27]`; still true of the live dashboard's own browser tab regardless of server load, though the check can no longer be done by opening a saved file with no Python process running at all, there is none to save since 2026-10-20 |
| `scripts/vio_log_extract.py`/`vio_log_plot.py` have no other consumer | Grepped across the repository, only the two plan documents describing past investigative use and the scripts' own docstrings reference them |

## 15. Deliberately not done

Auto detecting `alignment_mode` from `config.json`/`calibration.json` was considered and rejected, section 11 gives the reason, an explicit per invocation choice instead. A dependency scheduler that caches a loaded CSV across every metric that reads it was considered and rejected, section 5's closing paragraph gives the reason, `DataReader.load`'s own simplicity is preferred over a cache whose invalidation the user never asked for. A graph layout library for the covisibility graph was considered and rejected in favour of an adjacency heatmap, `go.Heatmap` needing no new dependency and the plot itself already caveated in `context/slam_analysis.md` section 5 as only meaningful with `mpVioDebugMode` on. Cross figure zoom linking, section 8/10, is deliberately scoped to plots whose x axis is `t_ns`, a genuine shared time base, `cost_curve`/`damping_trace`/`cost_split`/`mapper_cost_curve`/`mapper_damping_trace` plot a global solver iteration index instead and are not linked, since two different runs' iteration counts carry no common meaning a shared window would compare. Histograms, 3D trajectories, bar and funnel charts, box/violin plots and the diff table are not time series at all and are never tagged for linking either, `visualisation.py`'s own `link_zoom` parameter exists only on `time_series`/`multi_axis_time_series`/`categorical_timeline`, the three shapes that ever plot against `t_ns`.

## 16. `docker/analysis`, a dedicated image for `scripts/analysis`

`docker/ardupilot/Dockerfile` builds the runtime stack that flies SITL missions and records logs, a concern this dashboard has no part in, so adding `pandas`/`plotly`/`scipy`/`panel` to its existing `python3 -m pip install` layer was considered and rejected, folding an unrelated, growing dependency set (`cupy` in particular, which needs a CUDA toolkit `docker/ardupilot`'s own Ubuntu 22.04 CPU build chain has no reason to carry) into that image would only couple two things that should stay separate. A standalone image, `docker/analysis/Dockerfile`, built `FROM` an NVIDIA CUDA Ubuntu 22.04 base rather than layered onto `shandilya1998/nrt:camera`, carries `dashboard.py`'s own dependencies including `cupy`, and doubles as a general GPU capable development machine image for whatever else needs one later, matching `docker/dev`'s own role as a self contained editor image outside the `base -> ros -> pangolin -> camera -> ardupilot` chain rather than a further layer on it.

### 16.1 `docker/analysis/Dockerfile`

The Neovim, Claude Code and Antigravity CLI (`agy`) block is carried over verbatim from `docker/dev/Dockerfile:43-139`, the same block `docker/base/Dockerfile` and `docker/airsim/Dockerfile` already each carry their own copy of, so every image in this workspace that ships an in container IDE ships the identical toolchain. `docker/dev/Dockerfile` itself builds `FROM node:22-slim`, so it never needs to install Node, whereas `docker/base/Dockerfile` builds from plain `ubuntu:22.04` and bootstraps Node from NodeSource at its own lines 122-126 before the same `npm install -g` line runs. `docker/analysis/Dockerfile` is in the second situation, an NVIDIA CUDA base carries no Node either, so it repeats `docker/base/Dockerfile`'s own NodeSource bootstrap rather than `docker/dev/Dockerfile`'s assumption of a preinstalled one. The apt package list is `docker/base/Dockerfile`'s own list trimmed to what this image actually needs, dropping every Pangolin/OpenGL build dependency (`libglfw3-dev`, `libgl1-mesa-dev`, `libglu1-mesa-dev`, `libglew-dev`, `libeigen3-dev`, `lua5.2`, `ruby-dev`, `libperl-dev`) and every ROS lint/test package (`python3-flake8*`, `python3-pytest*`), since this image compiles nothing from this repository, it only runs `python3 scripts/analysis/dashboard.py`.

```dockerfile
# GPU capable image for scripts/analysis, standalone rather than a layer on
# base -> ros -> pangolin -> camera -> ardupilot, since it builds nothing
# from this repository and its own dependencies, cupy above all, have no
# place in the SITL runtime chain, context/slam_analysis.md and this plan's
# section 16 above.
FROM nvidia/cuda:12.4.1-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
ENV COLORTERM=truecolor
ENV SHELL=/bin/bash
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8
ENV LANGUAGE en_US:en

RUN echo 'Asia/Kolkata' > /etc/timezone && \
    ln -snf /usr/share/zoneinfo/Asia/Kolkata /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata gpg-agent && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    git \
    curl \
    wget \
    unzip \
    tar \
    gzip \
    fd-find \
    ripgrep \
    procps \
    ncurses-term \
    ncurses-base \
    openssh-client \
    clang \
    lldb \
    openjdk-17-jdk \
    locales \
    xclip \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-pylatexenc && \
    rm -rf /var/lib/apt/lists/*

# Debian's fd-find package ships the binary as `fdfind` (name clash with an
# unrelated `fd` package); symlink it so tools that look for the upstream
# binary name `fd` (e.g. agentic.nvim's file picker) find it.
RUN ln -s /usr/bin/fdfind /usr/bin/fd

# Without it, Neovim may struggle with special characters.
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen

ARG CACHEBUST=1
ARG TARGETARCH
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    curl -fsSL https://claude.ai/install.sh | bash; \
    else \
    echo "Skipping Claude Code install on $TARGETARCH"; \
    fi
ENV CLAUDE_CONFIG_DIR=/root/.claude
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    curl -fsSL https://antigravity.google/cli/install.sh | bash; \
    else \
    echo "Skipping Antigravity CLI install on $TARGETARCH"; \
    fi

# Node 22 via NodeSource, this base has no Node preinstalled unlike
# docker/dev's node:22-slim base, matching docker/base/Dockerfile's own
# bootstrap for the same reason.
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - && \
    apt-get install -y --no-install-recommends nodejs && \
    rm -rf /var/lib/apt/lists/*

RUN npm install -g @agentclientprotocol/claude-agent-acp agy-acp pyright bash-language-server dockerfile-language-server-nodejs tree-sitter-cli@0.25.10

RUN mkdir -p /opt/jdtls && \
    curl -L https://download.eclipse.org/jdtls/snapshots/jdt-language-server-latest.tar.gz -o /opt/jdtls/jdtls.tar.gz && \
    tar -xzf /opt/jdtls/jdtls.tar.gz -C /opt/jdtls && \
    rm /opt/jdtls/jdtls.tar.gz

WORKDIR /opt
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    NVIM_ASSET="nvim-linux-x86_64.tar.gz"; \
    NVIM_DIR="nvim-linux-x86_64"; \
    elif [ "$TARGETARCH" = "arm64" ]; then \
    NVIM_ASSET="nvim-linux-arm64.tar.gz"; \
    NVIM_DIR="nvim-linux-arm64"; \
    else \
    echo "Unsupported architecture: $TARGETARCH"; exit 1; \
    fi && \
    curl -fLO "https://github.com/neovim/neovim/releases/download/v0.11.0/$NVIM_ASSET" && \
    tar -xzf "${NVIM_ASSET}" && \
    rm "${NVIM_ASSET}" && \
    mv "${NVIM_DIR}" nvim

ARG FZF_VERSION=0.74.2
RUN if [ "$TARGETARCH" != "amd64" ] && [ "$TARGETARCH" != "arm64" ]; then \
    echo "Unsupported architecture: $TARGETARCH"; exit 1; \
    fi && \
    curl -fLO "https://github.com/junegunn/fzf/releases/download/v${FZF_VERSION}/fzf-${FZF_VERSION}-linux_${TARGETARCH}.tar.gz" && \
    tar -xzf "fzf-${FZF_VERSION}-linux_${TARGETARCH}.tar.gz" -C /usr/local/bin && \
    rm "fzf-${FZF_VERSION}-linux_${TARGETARCH}.tar.gz" && \
    fzf --version

ENV PATH="$PATH:/opt/nvim/bin"
RUN echo 'export PATH="$PATH:/opt/nvim/bin"' >> /root/.bashrc

RUN mkdir -p /root/.config/nvim /root/.config/nvim/lua/plugins/ /root/.config/nvim/ftplugin/
COPY init.lua /root/.config/nvim/init.lua
COPY core.lua /root/.config/nvim/lua/plugins/core.lua
COPY ai.lua /root/.config/nvim/lua/plugins/ai.lua
COPY git.lua /root/.config/nvim/lua/plugins/git.lua
COPY lang.lua /root/.config/nvim/lua/plugins/lang.lua
COPY lsp.lua /root/.config/nvim/lua/plugins/lsp.lua
COPY ui.lua /root/.config/nvim/lua/plugins/ui.lua

RUN sed -i 's/^# force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc && \
    echo "alias ls='ls --color=auto'" >> /root/.bashrc && \
    echo "alias grep='grep --color=auto'" >> /root/.bashrc && \
    echo "alias vim='nvim'" >> /root/.bashrc && \
    echo "alias vi='nvim'" >> /root/.bashrc && \
    echo "alias neovim='nvim'" >> /root/.bashrc

RUN mkdir -p /usr/share/fonts/truetype/jetbrains-mono && \
    wget -q https://github.com/ryanoasis/nerd-fonts/releases/download/v3.0.2/JetBrainsMono.zip && \
    unzip -q JetBrainsMono.zip -d /usr/share/fonts/truetype/jetbrains-mono && \
    rm JetBrainsMono.zip && \
    fc-cache -fv

RUN curl -LsSf https://astral.sh/ruff/install.sh | sh
ENV PATH="$PATH:/root/.local/bin"

# scripts/analysis's own runtime dependencies, section 13, plus cupy for the
# batched RPE sweep. cupy-cuda12x is matched to this image's own CUDA 12.x
# FROM line; the exact point release, 12.4.1 here, is not load bearing, any
# published nvidia/cuda:12.x-runtime-ubuntu22.04 tag satisfies it.
RUN python3 -m pip install --no-cache-dir -U pandas plotly scipy panel cupy-cuda12x

RUN nvim --headless "+Lazy! sync" +qa

WORKDIR /ws
CMD ["/bin/bash"]
```

The Neovim IDE block still needs `docker/dev/*.lua` copied alongside `Dockerfile` before `docker buildx build` runs, exactly as `djinn init docker nrt`'s own base image build already does at `djinn:75-77`, `setup_nvim_config_for_image_build $filepath` before and `cleanup_nvim_config_after_image_build $filepath` after, both already defined in `scripts/helpers/docker.sh` and reused here unchanged, only `$filepath` changes to `$NRT_WS/docker/analysis`.

### 16.2 `docker/scripts/start.sh`, the same runtime flags with a `--gpus` argument

`/ws/scripts/start.sh` is the generic container launcher `spawn_container_if_needed` already calls for every container that is not `airsim_container`/`airsim2_container`/`dev_container`, none of its `docker run` flags request a GPU. `sitl_ws/scripts/start.sh` shows the alternative, a hardcoded `--gpus=all`, correct there because AirSim always needs a GPU to render. `analysis_container` is neither case, `cupy` is an optional accelerator this stack already degrades from without a crash, section 13, so the flag must be configurable rather than hardcoded either way. `docker/scripts/start.sh` is `/ws/scripts/start.sh` verbatim with one addition, a fifth positional argument selecting `docker run`'s own `--gpus` value, defaulting to omitting the flag entirely so a machine with no NVIDIA Container Toolkit installed can still spawn this container.

```bash
#!/bin/bash

dir=$1
name="$2"
image="$3"
echo $name
echo $image
platform="$(uname -p)"
echo "host platform=$platform"

if [[ -n "$4" ]]; then
    platform="$4"
fi
echo "build platform=$platform"

if [[ "$platform" == "x86_64" ]]; then
    platform="linux/amd64"
elif [[ "$platform" == "aarch64" ]]; then
    platform="linux/arm64/v8"
fi

echo "platform=$platform"

# $5 selects docker run's own --gpus value. "none" (the default, also read
# whenever $5 is empty) omits the flag entirely so this script still runs on
# a machine with no NVIDIA Container Toolkit installed. "all" passes every
# GPU through, or a specific value such as "device=0" is forwarded verbatim.
gpus="${5:-none}"
gpu_args=()
if [[ "$gpus" != "none" ]]; then
    gpu_args=(--gpus "$gpus")
fi

XAUTH=/tmp/.docker.xauth
docker run -d --rm -it -v $dir:/ws/ \
    -v ./.ssh:/root/.ssh \
    -v /dev:/dev \
    -v $NRT_WS/.gemini:/root/.gemini \
    -v $NRT_WS/.claude:/root/.claude \
    -v $NRT_WS/.claude.json:/root/.claude.json \
    --device-cgroup-rule "c 81:* rmw" \
    --device-cgroup-rule "c 189:* rmw" \
    --device /dev/vchiq \
    -v /run/udev:/run/udev:ro \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTHORITY \
    -e TERM=xterm-256color \
    -e QT_GRAPHICSSYSTEM=native \
    -v $XAUTH:$XAUTH:rw \
    --platform=$platform \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --env="DISPLAY" --net=host \
    "${gpu_args[@]}" \
    --privileged --name $name $image bash
```

Every existing caller of `/ws/scripts/start.sh` is untouched, this is a new file, not an edit to the one every other container already uses.

### 16.3 `djinn init docker analysis` and `djinn up analysis`

`djinn init docker nrt`'s own dispatch at `djinn:65-213` is a chain of `if [[ "$3" == "nrt" ]]; then ... elif "dev" ... elif "airsim" ... elif "airsim2" ... elif "sitl" ... elif "kalibr" ... fi` inside `"$1" == "init"`/`"$2" == "docker"`. One more `elif` branch is added after the existing `kalibr` branch, mirroring the `nrt` base image build's own `setup_nvim_config_for_image_build`/`cleanup_nvim_config_after_image_build` pair since this image copies the same `docker/dev/*.lua` files:

```bash
            elif [[ "$3" == "analysis" ]]; then
                echo "============ BUILDING Analysis Image ============="
                image="$userName/nrt:analysis"
                filepath="$NRT_WS/docker/analysis"
                setup_nvim_config_for_image_build $filepath
                build_image $platform $image $filepath
                cleanup_nvim_config_after_image_build $filepath
```

`scripts/helpers/docker.sh`'s `spawn_container_if_needed` already special cases `dev_container` to call `docker/scripts/start_ide.sh` instead of the generic `scripts/start.sh`, `analysis_container` needs the same kind of special case, to call the new `docker/scripts/start.sh` above instead, so that every caller, both `djinn up analysis` and `djinn start slam-analysis`'s own auto spawn, reaches it through the one dispatch point every other container already goes through rather than duplicating a spawn path. `spawn_container_if_needed` gains a fourth, optional `gpus` parameter, empty by default, read only by the new branch, so its existing generic branch and every one of its existing callers, none of which pass a fourth argument, are unaffected:

```bash
spawn_container_if_needed() {
    name=$1
    image_name=$2
    platform=$3
    gpus=$4
    echo "Starting container of name: $name from image $image_name"

    if [[ -n "$(docker ps -a -q -f name="${name}")" ]]; then
        if [[ -n "$(docker ps -aq -f status=exited -f name="${name}")" ]]; then
            echo "Container Exists but stopped, restarting"
            docker start "${name}"
        else
            echo "Container exists and already present"
        fi
    else
        echo "Container does not exist creating now"
        if [[ ! -n "$platform" ]]; then
            platform="$(uname -p)"
        fi
        if [[ "${name}" == "airsim_container" ]]; then
            source $NRT_WS/sitl_ws/scripts/start.sh airsim
        elif [[ "${name}" == "airsim2_container" ]]; then
            source $NRT_WS/sitl_ws/scripts/start.sh airsim2
        elif [[ "$name" == "dev_container" ]]; then
            source $NRT_WS/docker/scripts/start_ide.sh dev_container shandilya1998/nrt:dev
        elif [[ "$name" == "analysis_container" ]]; then
            source $NRT_WS/docker/scripts/start.sh $NRT_WS "${name}" ${image_name} "${platform}" "${gpus:-none}"
        else
            source $NRT_WS/scripts/start.sh $NRT_WS "${name}" ${image_name} ${platform}
        fi
    fi
}
```

`djinn up`'s own dispatch at `djinn:217-259` gains an `analysis` branch, mirroring `up dev`'s spawn and git identity setup, since `analysis_container` has no ROS workspace of its own to build:

```bash
        elif [[ "$2" == "analysis" ]]; then
            echo "=============== Initialising Analysis Container ================"
            spawn_container_if_needed analysis_container shandilya1998/nrt:analysis "" "${3:-none}"
            iexec analysis_container "ls && echo \"Starting Container Setup\" && /ws/scripts/git-setup.sh $userEmail $userName"
```

`djinn up analysis` (bare) spawns the container with no GPU passthrough, `djinn up analysis all` (`$3` becomes `gpus`) enables it. `djinn start slam-analysis`, section 17 below, calls `spawn_container_if_needed analysis_container "shandilya1998/nrt:analysis"` with only two arguments, so it always reuses whatever container is already up rather than deciding the GPU flag itself, an operator who wants CuPy acceleration runs `djinn up analysis all` once beforehand.

## 17. `djinn start slam-analysis`

`dashboard.py` needs no ROS package sourcing, it only reads CSVs with `pandas` and serves the dashboard with `panel`, so it is executed with the plain `iexec` helper rather than `iexec_ros`/`iexec_sitl`/`iexec_sitl2`, none of whose overlay sourcing this script reads. `analysis_container`, section 16 above, is the container chosen, not `nrt_container`, since `nrt_container` is the SITL runtime stack `djinn start basalt`'s own debugging entry (`djinn:452`) runs Basalt in, a different concern from this analysis step, and `docker/analysis`'s own image is the one that actually carries `pandas`/`plotly`/`scipy`/`panel`/`cupy`. `analysis_container` mounts the whole workspace at `/ws` exactly as `nrt_container` does, `docker/scripts/start.sh -v $dir:/ws/` with `dir=$NRT_WS`, so `/ws/ros_ws/src/slam/ext/basalt` resolves identically and the invocation below is otherwise unchanged.

**Correction, 2026-10-20.** This originally called `dashboard.py` as a one shot batch script that wrote `dashboard.html` and exited on its own, reasoning that no `djinn kill slam-analysis` counterpart was therefore needed. `dashboard.py` is now a `panel serve` app, `plans/dashboard_live_migration.md`, a long running foreground process bound to a port, not a one shot batch step; the conclusion still holds for a different reason, `iexec` already runs `docker exec -it`, so `djinn start slam-analysis` blocks the calling shell on the attached server exactly as `djinn start basalt`'s own `gdb --ex run` entry already blocks on a long running foreground process, and Ctrl+C in that same terminal stops it, the same way an operator already stops that entry today, so a `djinn kill slam-analysis` counterpart is still not needed.

```bash
    elif [[ "$2" == "slam-analysis" ]]; then
        if [[ -z "$3" ]]; then
            echo "Usage: djinn start slam-analysis <absolute logdir path inside the container>"
        else
            echo "Starting SLAM analysis dashboard for $3 at http://localhost:5006"
            spawn_container_if_needed analysis_container "shandilya1998/nrt:analysis"
            iexec analysis_container "cd /ws/ros_ws/src/slam/ext/basalt && python3 -m panel serve scripts/analysis/dashboard.py --port 5006 --address 0.0.0.0 --allow-websocket-origin=\"*\" --args --logdir \"$3\""
        fi
```

This is inserted as one more `elif` branch inside `djinn`'s own `"$1" == "start"` block, directly after the existing `elif [[ "$2" == "basalt" ]]; then ... fi` branch and before the block's own closing `fi`, so every existing `start` subject, `tello`, `tello-orbslam3`, `sitl`, `sitl2`, `ardupilot_target_recorder`, `basalt`, is untouched.

Every piece of documentation the working conventions skill and this repository's own cross reference habit would otherwise leave stale is listed here, so Phase 5 updates all of it in the same change rather than only the ones later noticed missing. `djinn init docker analysis` and `djinn up analysis`, section 16 above, are two more new workflows needing the same treatment, listed alongside `djinn start slam-analysis` rather than in a separate table, since all three land in the same Phase 5 change.

| File | Update needed |
|---|---|
| `context/djinn-workflows.md` | Three new dedicated entries, `### djinn init docker analysis`, `### djinn up analysis [gpus]` and `### djinn start slam-analysis <logdir>`, in the same Call chain / Test row style as the neighbouring `### djinn init docker kalibr` and `### djinn start sitl2 [orbslam3\|basalt]` entries, mandatory per `working-conventions.md`'s "document every djinn workflow" rule |
| `context/djinn-cli.md` | The Subcommands table's `djinn init docker nrt\|dev\|airsim\|sitl\|kalibr` row gains `analysis`, `djinn up nrt\|dev\|airsim\|sitl_bridge\|kalibr` row gains `analysis`, and `start`'s row gains a new line for `slam-analysis`, taking a filesystem path as `$3` rather than a mode keyword the existing row's argument description already covers |
| `/ws/README.md` | The "Miscellaneous Commands" section's `djinn start sitl [orbslam3\|basalt]` example block gains a `./djinn start slam-analysis /ws/logs/<stamp>` line, and a `./djinn up analysis` / `./djinn up analysis all` pair alongside the other `djinn up` examples |
| `/ws/ARCHITECTURE.md` | Section 2's Docker image list gains `docker/analysis` as a standalone GPU capable image alongside `docker/dev`, not a further layer on `base -> ros -> pangolin -> camera -> ardupilot`; section 5's `start` subjects list, `tello`, `tello-orbslam3`, `sitl`, `sitl2`, `ardupilot_target_recorder`, gains `slam-analysis`, with a note that its argument is an absolute in container log directory path rather than a SLAM integration keyword |
| `scripts/analysis/README.md` | Written in Phase 5 alongside the code itself, section 15, mentions `djinn start slam-analysis <logdir>` as the recommended in container invocation next to the bare `python3 scripts/analysis/dashboard.py --logdir <logdir>` form |

## 18. Experiment-level parallelisation (`--workers`)

### Motivation

`Analysis.run()` (`analysis.py:123-144`) processes every experiment one at a time within each tab, and `DataReader.load()` (`data_reader.py:51-69`) is called roughly 35 times across the twelve `_analyse_<tab>` methods, each call re-reading one CSV per experiment sequentially. With CSVs running 27 to 43 MB per experiment across roughly 35 files per run, and three per-experiment loops (trajectory alignment/ATE/RPE, the excitation-covariance rolling window, the BoW PR-curve threshold sweep) doing real numeric work, a multi-experiment run on a multi-core host leaves most of its cores idle for no structural reason. This section parallelises across the experiment dimension without restructuring the tab-first architecture section 9 already established.

### High-level design

Two distinct bottlenecks need two distinct mechanisms, both scoped to the experiment dimension so neither touches the cross-tab shared state (`Analysis.shared_t_range`, a single `bokeh.models.DataRange1d` instance passed by reference into dozens of `visualisation.*` calls across every tab, `analysis.py:115`):

- **I/O**: `DataReader.load()`/`_scan_epochs()` dispatch their per-experiment `pd.read_csv` calls through a `ThreadPoolExecutor`. Pandas' C CSV parser releases the GIL during the actual parse, so threads give a real wall-clock win here with none of multiprocessing's pickling or start-up overhead.
- **CPU**: the three heavy per-experiment loops dispatch through a `ProcessPoolExecutor` owned by `Analysis.run()` for the duration of its tab loop. Two of the three (the excitation-covariance window and the BoW threshold sweep) are Python-level loops that hold the GIL between individual numpy calls, so only a process pool, which sidesteps the GIL entirely, gives genuine multi-core speedup for them. Each per-experiment loop body is extracted into a standalone, module-level, picklable function (plain numpy/pandas arguments and return values only, no `self`, no Bokeh object), submitted once per experiment, and merged back in deterministic `experiment_ids` order, never in worker-completion order, so `metrics.csv`'s row order and every figure's content stay identical regardless of which worker finishes first. The downstream figure-building code in each of the three tabs already only consumes the resulting per-experiment dicts, so it needs no change.

The other nine tabs are left untouched: their own per-experiment loops are lightweight pandas column selection, mean/std, and histogram counts dominated by the I/O that already happened by the time they run, not by their own compute, so pooling them would add process/thread overhead for no measurable return.

A correctness hazard, independent of which mechanism is used, had to be closed first. `visualisation._RUN_COLOURS` (`visualisation.py:15`) is a module-level dict populated lazily, in first-discovery order, by `_colour_for` (`visualisation.py:36-55`), relied on so that the same experiment id is drawn in the same colour on every tab. Concurrent first-touch of this plain, unlocked dict from different threads is a data race, and under multiprocessing each worker process would get its own independent copy, so colour assignment could diverge between workers. `visualisation.seed_run_colours(experiment_ids)` precomputes the whole mapping, once, deterministically, from the already-sorted `DataReader.experiment_ids`, before any thread or process pool work starts, turning `_colour_for` into a read-only cache for the remainder of the run regardless of what touches it afterwards.

`docker/analysis` (section 16) installs `cupy-cuda12x` on a CUDA base image; when it is run with `--gpus` passed through, `metrics.relative_pose_error_sweep` (`metrics.py:55-65`) defaults to the GPU automatically via `xp = xp or (cp if _HAVE_CUPY else np)`, with no explicit opt-in anywhere in the call chain. Confirmed absent from the dev container itself (`python3 -c "import cupy"` raises `ModuleNotFoundError` there), so this only matters on `analysis_container`. Running `--workers > 1` on such a host means every pooled worker process independently acquires its own CUDA context the first time the trajectory worker calls into the RPE sweep, which can contend for a single GPU. The decision, made explicitly rather than by default, is to leave the GPU path exactly as it already behaves and only document the risk here and in the `--workers` CLI help text, rather than forcing CPU numpy inside pooled workers or clamping `--workers` to 1 whenever `cupy` is importable. `TrajectoryAnalysis.rpe(..., xp=...)` / `relative_pose_error_sweep(..., xp=...)` already accept an explicit override, so a future session hitting real contention can force `xp=np` inside the trajectory worker specifically when pooled, without needing a new interface.

### Detail

**`data_reader.py`**: `DataReader.__init__(self, logdir, workers=None)` resolves `self.workers = workers if workers is not None else min(len(self.experiment_ids), os.cpu_count() or 1)` immediately after `self.experiment_ids` is built, then constructs `self._io_pool = ThreadPoolExecutor(max_workers=self.workers) if self.workers > 1 else None` before calling `self._scan_epochs()`, since that call already needs the pool. `_scan_epochs`'s per-experiment body (the `for fname in os.listdir(exp_dir): ...` block) becomes `_scan_epoch_one(self, exp) -> tuple[str, int | None]`; `_scan_epochs` itself dispatches it through `self._io_pool.map(...)` when pooled, else the original sequential generator, and folds the `(exp, m)` pairs into the same `dict[str, int]` as before. `load`'s per-experiment body becomes `_load_one(self, name, exp) -> tuple[str, pd.DataFrame | None]`, dispatched the same way, folded into the same `dict[str, pd.DataFrame]` return shape. `load_json` is left sequential, too few and too small to be worth pooling. No caller in `analysis.py` changes, since the return shape and the fact that it is a plain dict keyed by experiment id, with no ordering guarantee callers ever relied on, are unchanged.

**`visualisation.py`**: `seed_run_colours(experiment_ids: list[str]) -> None`, added directly after `_colour_for`, is a four-line loop that calls `_colour_for(exp)` for every id in the given list purely for its caching side effect.

**`analysis.py`**: `Analysis.__init__(self, reader, config, out_dir, workers=None)` adds `self.workers = workers if workers is not None else reader.workers`, calls `visualisation.seed_run_colours(self.reader.experiment_ids)` immediately after constructing `self.shared_t_range`, and adds `self._pool = None`, live only for the duration of `run()`. `run()` wraps its existing `for tab in TABS:` loop in `with (ProcessPoolExecutor(max_workers=self.workers) if self.workers > 1 else nullcontext()) as pool:`, assigning `self._pool = pool` (so it is `None` precisely when running sequentially) before the loop and resetting it to `None` after, before the figure-writing and `metrics.csv` code that follows. The pool is created fresh inside `run()`, never at import time or in a constructor, so no CuPy array can exist in the parent process before Linux's default `fork` start method duplicates it, keeping `fork` safe here.

Three call sites dispatch through `self._pool` when present, or call the same function directly otherwise, merging results by iterating `self.reader.experiment_ids` rather than completion order:

- Trajectory tab (`_analyse_trajectory`, `analysis.py:169-240`): a new module-level `_trajectory_worker(exp, t_ns, gt_xyz, gt_q, est_xyz, est_q, alignment_mode, cfg, yaw_offset) -> dict` builds its own fresh `TrajectoryAnalysis(alignment_mode)` and runs the body currently at `analysis.py:189-240` (`set_alignment`/`apply`/`ate`/`rpe` twice/`drift_proxies`/`reset_alignment`), returning a dict keyed `est_xyz_aligned`, `ate`, `err_xyz`, `rpe_rmse_curve`, `rpe_rmse_per_delta_series`, `rpe_length_per_delta_series`, `yaw_offset`, `drift_proxies`, each `None` when its own `cfg.get(...).get("enabled")` gate is false. `_analyse_trajectory` keeps the `df.dropna`/empty check and the `t_ns`/`vec(...)` extraction, since those decide whether to submit at all, and resolves `yaw_offset` from `vio_init` in the main process rather than shipping that whole dict into the worker. Everything from `analysis.py:242` onward, which only ever consumed `est_xyz_per_run`/`err_per_run`/etc., is unchanged.
- Excitation-covariance loop (inside `_analyse_vio_optimisation`, `analysis.py:1002-1030`): a new `_excitation_worker(imu_df, st_df, g, window_s) -> tuple[pd.Series, np.ndarray] | None` contains the body at `analysis.py:1016-1029` (`align_frames_on_t_ns` / `DataReader.vec` (a `@staticmethod`) / `Rotation.from_quat` / `np.einsum` / `metrics.excitation_covariance`), returning `None` on an empty merge. The method keeps the `exp not in vio_state or exp not in vio_init` guard and the `g = self.reader.vec(...)` lookup ahead of dispatch.
- BoW PR-curve loop (inside `_analyse_mapping_matching`, `analysis.py:1284-1296`): no extraction needed, `metrics.bow_pr_curve` is already a free function with plain-data arguments and return values; only the dispatch/merge pattern around the existing call changes.

A small shared one-line helper exposing `.result()` for an already-computed value lets the merge loop at all three sites be written once per site regardless of whether `self._pool` is active, rather than duplicating a branch at each site.

**`dashboard.py`**: `_parse_args` gains `ap.add_argument("--workers", type=int, default=None, help=...)`, its help text stating the default (`min(experiments, os.cpu_count())`), that `1` disables pooling entirely, and the GPU/CuPy contention caveat above. `_args.workers` (`None` unless the user set it) flows unchanged into both `DataReader(_args.logdir, workers=_args.workers)` and `Analysis(_reader, _config, _args.logdir, workers=_args.workers)`; the canonical default is computed exactly once, inside `DataReader.__init__`, and `Analysis` reuses `reader.workers` whenever its own `workers` argument is `None`, so there is no duplicate default-computation path.

### Backwards compatibility impact

Every new parameter (`workers` on `DataReader.__init__` and `Analysis.__init__`, `--workers` on `dashboard.py`) defaults to `None`, resolving to the same sensible worker count rather than requiring any caller to opt in. `--workers 1` reproduces today's fully sequential behaviour exactly, call for call, since every dispatch site falls back to calling the extracted function directly rather than through a pool whenever `self._pool`/`self._io_pool` is `None`. No existing caller of `DataReader.load()`, `DataReader._scan_epochs()`, or any `_analyse_<tab>` method changes its invocation, since the return shapes (`dict[str, pd.DataFrame]`, `dict[str, int]`, the per-tab figure/metric side effects) are identical to before. `metrics.csv` and every figure's numeric content are required to be identical to a pre-change sequential run (verification item 1 below), with only possible floating point noise ruled out by construction since no change touches the arithmetic itself, only when and where it runs.

### Validation strategy

1. Run `panel serve scripts/analysis/dashboard.py --args --logdir <dir> --workers 1` and again with `--workers <N>` (`N > 1`) against the same `--logdir`; diff `metrics.csv` against each other and against a pre-change baseline run. `metrics.pivot_metrics` (`metrics.py:180-191`) already pivots and sorts by `(category, name, type)` and by experiment column name, so row and column order should match independent of merge order, a useful independent cross-check that the deterministic `experiment_ids`-order merge was implemented correctly rather than merely relied upon.
2. Run `--workers <N>` twice in a row against the same `--logdir` and diff the two outputs directly against each other, the most direct test that worker-completion-order non-determinism has been fully neutralised.
3. Spot-check a couple of multi-experiment figures (`trajectory_3d`, `bow_pr_curve`) across a `--workers 1` and a `--workers N` run to confirm each experiment keeps the same colour, confirming the `seed_run_colours` fix.
4. Temporarily dump `est_xyz_per_run`/`t_ns_per_run`/`rpe_rmse_series`/the excitation-covariance `y` dict/the BoW `x_per_run`/`y_per_run` to `.npz` right before figure-building in both a `--workers 1` and a `--workers N` run and `np.allclose`-compare, checking the actual numeric payload directly rather than diffing rendered HTML.
5. `Analysis.run()` already logs per-tab timing (`analysis.py:125-132,144`); compare the `_analyse_trajectory` / `_analyse_vio_optimisation` / `_analyse_mapping_matching` lines and the total `Analysis.run() total` line across `--workers` values of `1, 2, 4, os.cpu_count()` on the same `--logdir` to confirm an actual wall-clock speedup. Since `dashboard.py` runs `Analysis.run()` eagerly before `_build(...)`, this total is also the page's time-to-first-byte.
6. While a `--workers > 1` run is in progress, check `ps -eLf | grep panel` to confirm pool worker processes appear only during the trajectory/vio_optimisation/mapping_matching tabs and are gone once `Analysis.run()` returns, confirming the pool's lifetime is correctly scoped to the `with` block inside `run()`.
7. None of `pandas`/`plotly`/`scipy`/`panel` are installed in the dev container (confirmed, `python3 -c "import pandas"` raises `ModuleNotFoundError` there, consistent with section 13), so items 1 through 6 can only run inside `analysis_container` (`djinn start slam-analysis`); the dev container pass is limited to `python3 -m py_compile` on each edited file plus a direct reading of the diff.

### Deliberately not done

The tab-first outer loop in `Analysis.run()` is not restructured into an experiment-first one. That would touch all twelve `_analyse_<tab>` methods to parallelise nine tabs whose own per-experiment loops are not CPU-bound in the first place, for no measurable benefit over pooling `DataReader.load()`'s I/O, which every tab already benefits from regardless of its own compute shape. Forcing `xp=np` inside the trajectory worker whenever pooled, to eliminate the GPU-contention risk described above outright rather than merely documenting it, is left to a future session, since `docker/analysis` is the only environment where `cupy` is present and the decision trades away GPU acceleration specifically on that one tab whenever the pool is active, a tradeoff the user chose not to make by default.

## 19. Gravity and scale tab, coverage of every logged column, and a lift off time reference

Implemented 2026-11-11.

### Motivation

An audit on 2026-11-11 of `/ws/logs/slam/2026-11-11_19-54-39` against `analysis.py`, `metrics.py` and `trajectory_analysis.py` found four logged CSVs that no tab loads, `vio_gravity_scale`, `mapper_iter_summary`, `mapper_stereo` and `vo_stationary` (the last two empty in that run), and 199 columns of the loaded CSVs that no figure or scalar reads. `vio_gravity_scale.csv`, the per frame record of the gravity and scale period (`SqrtKeypointVioEstimator::LogGravityScale`), is entirely unvisualised, as are the quantities the period's analysis in `context/dynamic_marginalisation.md` §9 to §11 had to derive by hand. Those are the scale ratio and the body frame tilt error against ground truth over time, the accelerometer bias components (only `ba_norm` is drawn, `state_trace`), the per frame position jump that exposes re-centring about the origin, the mapper pose update errors (`vio_pose_update.max_trans_err`, `max_rot_err`), and the solver's convergence (`vio_solver_summary.converged`, `num_it_rejected`). Take offs also happen 3 s to 7 s into each run, so comparing take offs on mission relative time misaligns them.

### High-level design

Three additions, none of which changes an existing figure or scalar.

1. A `gravity_scale` tab, placed after `vio_optimisation`, that draws every column of `vio_gravity_scale.csv` and the physical quantities above, with per run scalars of the period.
2. A `raw_logs` tab that draws every numeric column of every CSV found in any experiment, one stacked figure per CSV, so that nothing logged is ever invisible again, including CSVs added in future. Columns that are constant within every run (thresholds, enable flags) become scalars instead of flat plots.
3. An optional `time_reference` in `config.json`, `"mission"` (default, today's behaviour) or `"liftoff"`, which offsets every experiment's `t_ns` by its own lift off instead of its first logged row.

The new physical metrics are written with NumPy only, quaternions included, so they can be tested in the dev container, which has NumPy but not pandas, SciPy, Bokeh or Panel (checked 2026-11-11).

### Detail

#### Change 1. `metrics.py`, pure functions for the period

Appended after `config_diff`. Quaternions are $(x, y, z, w)$ as logged by `Logger::AddGtEval` (Eigen `coeffs()`).

```python
G_NORM = 9.81


def _rotate(q_xyzw: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotates each row of v by the matching unit quaternion, (x, y, z, w)."""
    u, w = q_xyzw[:, :3], q_xyzw[:, 3:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def _conj(q_xyzw: np.ndarray) -> np.ndarray:
    return np.concatenate([-q_xyzw[:, :3], q_xyzw[:, 3:]], axis=1)


def _angle_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    c = (a * b).sum(1) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1))
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def tilt_error_deg(q_est: np.ndarray, q_gt: np.ndarray) -> np.ndarray:
    """Angle between the estimated and true body frame gravity directions."""
    z = np.tile([0.0, 0.0, 1.0], (len(q_est), 1))
    return _angle_deg(_rotate(_conj(q_est), z), _rotate(_conj(q_gt), z))


def yaw_error_deg(q_est: np.ndarray, q_gt: np.ndarray) -> np.ndarray:
    """Heading difference of the body x axis projected on the horizontal plane."""
    x = np.tile([1.0, 0.0, 0.0], (len(q_est), 1))
    fe, fg = _rotate(q_est, x), _rotate(q_gt, x)
    d = np.degrees(np.arctan2(fe[:, 1], fe[:, 0]) - np.arctan2(fg[:, 1], fg[:, 0]))
    return (d + 180.0) % 360.0 - 180.0


def liftoff_t_ns(t_ns: np.ndarray, gt_z: np.ndarray, threshold_m: float) -> int | None:
    """First t_ns at which the true height leaves its first value by threshold_m."""
    idx = np.flatnonzero(np.abs(gt_z - gt_z[0]) > threshold_m)
    return int(t_ns[idx[0]]) if idx.size else None


def windowed_scale_ratio(t_ns: np.ndarray, est_xyz: np.ndarray, gt_xyz: np.ndarray,
                         window_s: float, min_disp_m: float) -> np.ndarray:
    """|estimated displacement| / |true displacement| over the trailing window,
    NaN where the true displacement is below min_disp_m."""
    j = np.searchsorted(t_ns, t_ns - int(window_s * 1e9))
    dg = np.linalg.norm(gt_xyz - gt_xyz[j], axis=1)
    de = np.linalg.norm(est_xyz - est_xyz[j], axis=1)
    out = np.full(len(t_ns), np.nan)
    ok = dg > min_disp_m
    out[ok] = de[ok] / dg[ok]
    return out


def position_jump(t_ns: np.ndarray, p: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Per frame displacement not explained by the mean of the two velocities."""
    dt = np.diff(t_ns) * 1e-9
    pred = 0.5 * (v[1:] + v[:-1]) * dt[:, None]
    return np.r_[0.0, np.linalg.norm(p[1:] - p[:-1] - pred, axis=1)]


def block_update_mask(sigma_step: np.ndarray, tilt_step_deg: np.ndarray,
                      tilt_tol_deg: float = 1e-3) -> np.ndarray:
    """Frames on which the gravity and scale block moved. tilt_step_deg is an
    acos of a dot product, which turns rounding of 1e-16 into about 1e-6
    degrees on held frames, hence the tolerance."""
    return (np.abs(sigma_step) > 1e-9) | (np.abs(tilt_step_deg) > tilt_tol_deg)


def bias_equivalent_tilt_deg(ba_xyz: np.ndarray) -> np.ndarray:
    """Tilt that the horizontal accelerometer bias trades against at rest."""
    return np.degrees(np.arcsin(np.clip(np.linalg.norm(ba_xyz[:, :2], axis=1) / G_NORM, 0.0, 1.0)))
```

Two functions beyond the listing were added in implementation, `finite_difference_velocity(t_ns, xyz)`, `np.gradient` of the true positions for the speed ratio, and `G_NORM` is used only by `bias_equivalent_tilt_deg`.

#### Change 2. `data_reader.py`, lift off reference and CSV discovery

`DataReader.__init__(self, logdir, workers=None, time_reference="mission", liftoff_threshold_m=0.15)`. After `_scan_epochs()`, when `time_reference == "liftoff"`, each experiment's epoch is replaced by its lift off from `gt_eval.csv` (`metrics.liftoff_t_ns` on `t_ns` and `p_gt_2`), keeping the mission epoch for an experiment without ground truth or without a lift off. In both modes `self.liftoff_t_ns[exp]` holds the lift off relative to the active epoch (0 in lift off mode), which the new tab uses for its scalars.

```python
    def csv_names(self) -> list[str]:
        """Every CSV base name present in at least one experiment, sorted."""
        names = set()
        for exp in self.experiment_ids:
            for f in os.listdir(os.path.join(self.logdir, exp)):
                if f.endswith(".csv"):
                    names.add(f[:-4])
        return sorted(names)
```

`metrics.py` imports nothing from `data_reader.py`, so `data_reader.py` may import `metrics.liftoff_t_ns` without a cycle.

`DataReader._load_one` also returns `None` for a zero byte file, as it did for a missing or header only one. The `Logger` writes a header only with its first flush, so a table that never flushed is empty, and `pd.read_csv` raises `EmptyDataError` on it. No tab loaded such a file before, but `raw_logs` loads every CSV, and a run still being recorded has more of them, which is how the defect surfaced on 2026-11-11 (`vio_init.csv` of a run in progress). The fix then exposed a second defect in an existing tab. `vio_marg_nullspace.csv` is empty in every run with the gravity and scale period active (`logMargNullspace` is gated on `!mpGsActive`), so `_analyse_vio_optimisation` built its eigenspectrum and nullspace response figures with no runs and `visualisation._legend_outside` raised `IndexError`. Both blocks now require data, and `_legend_outside` returns early for a figure without a legend.

#### Change 3. `dashboard.py`

```python
_reader = DataReader(_args.logdir, workers=_args.workers,
                     time_reference=_config.get("time_reference", "mission"),
                     liftoff_threshold_m=_config.get("liftoff_threshold_m", 0.15))
```

#### Change 4. `analysis.py`, `_analyse_gravity_scale`

`TABS` gains `"gravity_scale"` after `"vio_optimisation"`. Every figure is skipped when its source CSV is absent in every experiment, so runs without the period produce an empty tab, which `dashboard._build` already hides (`if figs`). Figures, each behind its own `enabled` flag, every time series on `self.shared_t_range`:

| Figure | Source | Shape |
|---|---|---|
| `block_step` | `vio_gravity_scale`, update frames only (`block_update_mask`) | `multi_axis_time_series` of `sigma_step`, `tilt_step_deg`, `jump_cost` |
| `block_cumulative` | `vio_gravity_scale`, every frame | `multi_axis_time_series` of `scale_cumulative`, `tilt_cumulative_deg`, `epoch` |
| `swap_events` | `vio_gravity_scale`, frames with a swap only | `categorical_timeline` of `swap_event`, 1 half reset, 2 upward, 3 downward |
| `visual_prior_admission` | `vio_gravity_scale` | `multi_axis_time_series` of `visual_imu_full`, `visual_imu_rot`, `visual_imu_scale_col` |
| `physical_scale_gravity` | `gt_eval` aligned with `vio_state` (`metrics.align_frames_on_t_ns`) | `multi_axis_time_series` of windowed scale ratio, tilt error, yaw error, speed ratio $|\mathbf{v}_\text{est}| / |\mathbf{v}_\text{gt}|$ (true velocity by finite differences of `p_gt`) |
| `bias_components` | `vio_state` | `multi_axis_time_series` of `ba_0..2`, bias equivalent tilt, `bg_0..2` |
| `position_jump` | `vio_state`, with `vio_gravity_scale` update frames as overlay markers (`time_series(..., marker_index_per_run=...)`) | `time_series` |
| `pose_update_gate` | `vio_pose_update` | `multi_axis_time_series` of `applied`, `rejected`, `max_trans_err`, `max_rot_err` |
| `solver_health` | `vio_solver_summary` | `multi_axis_time_series` of `num_it`, `num_it_rejected`, `converged` |

Scalars, category `gravity_scale`, per experiment: lift off time (s), first block update after lift off (s), update rate (1/s), swaps in total and on held frames, largest $|\delta\sigma|$ and tilt step, final `scale_cumulative`, largest $|$`tilt_cumulative_deg`$|$, largest tilt error after lift off and the tilt error at 10 s, median scale ratio between 5 s and 30 s after lift off, largest position jump on update and on held frames, and the rejected fraction of pose updates.

#### Change 5. `analysis.py`, `_analyse_raw_logs`

`TABS` gains `"raw_logs"` before `"cross_run"`.

```python
    def _analyse_raw_logs(self, cfg):
        figs = []
        if not cfg.get("enabled"):
            return figs
        exclude = set(cfg.get("exclude", []))
        max_points = int(cfg.get("max_points_per_run", 20000))
        per_plot = int(cfg.get("height_per_subplot", 160))
        for name in self.reader.csv_names():
            if name in exclude:
                continue
            data = {e: d for e, d in self.reader.load(name).items() if "t_ns" in d.columns}
            if not data:
                continue
            cols = sorted({c for d in data.values() for c in d.columns
                           if c != "t_ns" and pd.api.types.is_numeric_dtype(d[c])})
            plotted = []
            for c in cols:
                if all(c not in d or d[c].nunique(dropna=True) <= 1 for d in data.values()):
                    for e, d in data.items():
                        if c in d and d[c].notna().any():
                            self._add_metric("raw_logs", e, f"{name}.{c}", "value", float(d[c].dropna().iloc[0]))
                else:
                    plotted.append(c)
            if not plotted:
                continue
            step = {e: max(1, len(d) // max_points) for e, d in data.items()}
            t_ns = {e: d["t_ns"].iloc[::step[e]] for e, d in data.items()}
            quantities = {c: {e: d[c].iloc[::step[e]] for e, d in data.items() if c in d} for c in plotted}
            figs.append((f"raw_{name}", visualisation.multi_axis_time_series(
                t_ns, quantities, f"{name}.csv, every non constant numeric column", "t_ns", "value",
                x_range=self.shared_t_range, width=cfg.get("width", 1080),
                height=min(per_plot * len(plotted), 6000))))
        return figs
```

`analysis.py` does not import pandas today, so the change adds `import pandas as pd` beside `import numpy as np`. `multi_axis_time_series` takes one `t_ns` per run for all subplots, which holds within one CSV. A column missing from one experiment is simply absent from that subplot. Subsampling by a stride keeps each run at or below `max_points_per_run` points per subplot, which only binds for `map_rehost` (150 700 rows) and `of_track_death` (105 545 rows) in the kept runs.

#### Change 6. `config.json`

```json
  "time_reference": "mission",
  "liftoff_threshold_m": 0.15,
  "gravity_scale": {
    "block_step": {"enabled": true, "width": 1080, "height": 750},
    "block_cumulative": {"enabled": true, "width": 1080, "height": 750},
    "swap_events": {"enabled": true, "width": 1080, "height": 400},
    "visual_prior_admission": {"enabled": true, "width": 1080, "height": 750},
    "physical_scale_gravity": {"enabled": true, "window_s": 2.0, "min_disp_m": 0.3, "min_speed_mps": 0.3,
                               "width": 1080, "height": 1000},
    "bias_components": {"enabled": true, "width": 1080, "height": 1400},
    "position_jump": {"enabled": true, "width": 1080, "height": 400},
    "pose_update_gate": {"enabled": true, "width": 1080, "height": 1000},
    "solver_health": {"enabled": true, "width": 1080, "height": 750},
    "scalars": {"enabled": true, "median_ratio_window_s": [5, 30], "tilt_at_s": 10}
  },
  "raw_logs": {"enabled": true, "exclude": [], "max_points_per_run": 20000, "height_per_subplot": 160, "width": 1080},
```

`time_reference` ships as `"mission"`, so the shipped default is unchanged. For take off comparisons, set `"liftoff"` in a copy passed with `--args --logdir ... --config <copy>`.

#### Change 7. Documentation and context

`scripts/analysis/README.md` gains the two tabs and the two top level keys in its `config.json` section. `context/slam_analysis.md` gains the coverage audit, the lift off reference and the held frame tolerance of `block_update_mask`. `/ws/context/djinn-workflows.md` is unchanged, since `djinn start slam-analysis` is unchanged.

### Backwards compatibility impact

No existing figure, scalar or tab changes. `time_reference` defaults to `"mission"`, which reproduces today's epochs exactly. The new tabs add figures to `figures/gravity_scale/` and `figures/raw_logs/` and rows to `metrics.csv` under two new categories, and `metrics.pivot_metrics` sorts by category, so existing rows keep their relative order. `Analysis.run()` takes longer by the raw logs tab, which can be disabled or narrowed with `exclude`. A run without the period yields an empty `gravity_scale` tab, which the dashboard hides.

### Validation strategy

1. In the dev container, `python3 -m py_compile` of every edited file. Passed 2026-11-11 for `metrics.py`, `data_reader.py`, `analysis.py` and `dashboard.py`, and `config.json` parses.
2. In the dev container, the NumPy only functions of Change 1, executed from the parsed source of `metrics.py` without importing pandas, against the hand analysis of `context/dynamic_marginalisation.md` §11 on `/ws/logs/slam/2026-11-11_19-43-15`: tilt error 42.7° at 3.91 s after lift off, 121 block updates between 2 s and 65 s, swaps 7 on update and 3 on held frames, lift off at 670.0 s. Run 2026-11-11 and matched, lift off at 670.002 s, 121 updates, swaps 7 and 3, mean position jumps of 2.312 m on update and 0.013 m on held frames between 2 s and 6 s, a yaw error of $-3.9°$ at 50 s and a scale ratio of 1.60 at 10 s, with synthetic checks giving zero tilt and 45° yaw for a pure 45° yaw and 30° tilt for a 30° roll. Item 3 is outstanding.
3. In `analysis_container`, `djinn start slam-analysis /ws/logs/slam` on the four kept runs with `time_reference` `"mission"` and then `"liftoff"`, confirming every tab renders, `metrics.csv` gains the two categories, and the lift off mode puts each lift off at $t = 0$.

### Deliberately not done

No new figure in an existing tab, so existing tabs and their tests compare unchanged. No per column hand tuned plots for the 199 unread columns, which the raw logs tab covers generically. No automatic choice of `time_reference`, since the mission reference remains right for replays of one dataset.

## 20. Delayed marginalisation tab

Implemented 2026-11-14.

### Motivation

`plans/delayed_marginalisation.md` made delayed marginalisation the only mechanism of the gravity and scale period, and its estimator logs every marginalisation of the period to `vio_delayed_marg.csv`, one row per marginalisation with the store's size, the jobs started, installed and discarded, their durations, the replacement rule's inputs and the result of every coarse optimisation. No tab read that file except `raw_logs`, which draws every column as an undifferentiated stack, so the questions that diagnose the mechanism had no direct answer. When was the prior replaced, by which kind of job, and with what effect on the scale? How long did each job take in the worker against the frame period, how long did the VIO thread spend installing it or waiting for it, and how many frames passed between starting a job and installing its prior? What drove each readvance, and what did each coarse optimisation estimate? The `swap_events` figure of the `gravity_scale` tab also still named only the swap codes 1 to 3 of dynamic marginalisation, while the delayed mode writes 4 (fold), 5 (coarse correction) and 6 (readvance).

### High-level design

A `delayed_marginalisation` tab, placed after `gravity_scale`, built from `vio_delayed_marg.csv`, `vio_gravity_scale.csv` and each run's `config.json`, with ten figures and per run scalars. Every figure is `t_ns` based and shares the dashboard's linked range, except the histogram of job durations. Event detection and the pairing of a job's start with its installation are NumPy only functions in `metrics.py`, so they can be tested in the dev container, which has NumPy but not pandas, Bokeh or Panel (checked again 2026-11-14).

The tab reads two schemas of `vio_delayed_marg.csv`. Logs written before Change 16 of `plans/delayed_marginalisation.md` carry `coarse_triples` and set `installed` only for readvances, since a finished coarse job always installed. Logs since Change 16 carry `coarse_spans`, set `installed` on a coarse job that installed, and leave it zero on one discarded for lack of keyframe factors or, since Change 17, by its deadline, with `coarse_bg`, `coarse_max_vel_inc`, `coarse_iterations`, `coarse_timed_out` and `wait` added. Every optional column is drawn only where it exists.

| Figure | Content | Rows |
|---|---|---|
| `dm_events` | readvance started, coarse job started, readvance installed, coarse correction installed, coarse job discarded, delayed phase begins | event rows |
| `dm_job_timing` | job duration in the worker, the frame period before the row as the job's budget, the VIO thread's wait, installation with catch up, steps readvanced at installation | rows where a job finished |
| `dm_install_lag` | marginalisations from a job's start to its installation or discard | rows where a job started |
| `dm_store_timing` | store capture, push and advance per marginalisation | every row |
| `dm_trigger` | log scale and tilt offset of the block from the prior's anchor, with each run's readvance thresholds in the subplot titles | every row |
| `dm_store` | stored steps, records, coarse phase flag | every row |
| `dm_coarse_jobs` | every coarse output column present | rows where a coarse job finished |
| `dm_smoothing` | largest position increment of the smoothing pass | rows where a readvance installed |
| `dm_scale_at_events` | cumulative scale of the correction log with every installation marked | `vio_gravity_scale.csv` rows |
| `dm_job_duration_histogram` | distribution of job durations | finished jobs |

The scalars are the count of every event, when the delayed phase begins from the epoch and from lift off, readvances per minute in the delayed phase, the median, 95th percentile and maximum job duration, the number of jobs longer than their frame period, the largest installation cost, the largest wait and the number of frames that waited, the median and largest store cost, the mean and largest installation lag, the number of coarse jobs the deadline stopped, the largest smoothing increment, and the largest store and record counts.

### Detail

#### Change 1. `metrics.py`, events and installation lag

Appended at the end of the file. A job that finishes is marked by `installed` or `coarse` on the row of the marginalisation that installed or discarded it. Only one job runs at a time, so each start pairs with the next finish, strictly after it for an asynchronous job, since a row can install the previous job and start a new one, and on the same row for a synchronous job.

```python
# Delayed marginalisation, plans/analysis_module.md section 20. Event labels of
# vio_delayed_marg.csv in the order the dashboard lists them.
DM_EVENTS = (
    "readvance started",
    "coarse job started",
    "readvance installed",
    "coarse correction installed",
    "coarse job discarded",
    "delayed phase begins",
)


def delayed_marg_events(started: np.ndarray, installed: np.ndarray, coarse: np.ndarray,
                        coarse_phase: np.ndarray, coarse_install_flagged: bool) -> dict[str, np.ndarray]:
    """Row indices of each event of vio_delayed_marg.csv. started is 1 for a
    readvance and 2 for a coarse job, coarse is 1 on the row where a coarse
    job finished. Since Change 16 of plans/delayed_marginalisation.md a coarse
    job that installed also sets installed, so coarse without installed means
    it was discarded (no keyframe factor, or its deadline). Logs written before
    that (column coarse_triples instead of coarse_spans) always installed a
    finished coarse job, which coarse_install_flagged False reproduces."""
    started, installed, coarse = (np.asarray(a) for a in (started, installed, coarse))
    phase = np.asarray(coarse_phase)
    coarse_done = coarse > 0
    coarse_ok = coarse_done & (installed > 0) if coarse_install_flagged else coarse_done
    return {
        "readvance started": np.flatnonzero(started == 1),
        "coarse job started": np.flatnonzero(started == 2),
        "readvance installed": np.flatnonzero((installed > 0) & ~coarse_done),
        "coarse correction installed": np.flatnonzero(coarse_ok),
        "coarse job discarded": np.flatnonzero(coarse_done & ~coarse_ok),
        "delayed phase begins": np.flatnonzero((phase[:-1] > 0) & (phase[1:] == 0)) + 1,
    }


def job_install_lag(started: np.ndarray, installed: np.ndarray, coarse: np.ndarray,
                    asynchronous: bool) -> tuple[np.ndarray, np.ndarray]:
    """Rows at which a job started and the number of marginalisations until
    the row that installed or discarded it. Only one job runs at a time, so
    each start pairs with the next finish. An asynchronous job installed in a
    later call shares its row with any job started in that call, hence a
    finish counts only strictly after its start, while a synchronous job
    finishes in the row that started it."""
    finished = np.flatnonzero((np.asarray(installed) > 0) | (np.asarray(coarse) > 0))
    starts = np.flatnonzero(np.asarray(started) > 0)
    j = np.searchsorted(finished, starts, side="right" if asynchronous else "left")
    ok = j < len(finished)
    return starts[ok], finished[j[ok]] - starts[ok]
```

#### Change 2. `analysis.py`, the tab

`"delayed_marginalisation"` is inserted into `TABS` after `"gravity_scale"`, and the method below follows `_analyse_gravity_scale`. Each run's `vio_gs_dm_async`, `vio_gs_dm_thresh_scale` and `vio_gs_dm_thresh_tilt_deg` are read from its `config.json` under cereal's `value0`, with the estimator's defaults where a key is missing. The title of `gs_swap_events` in `_analyse_gravity_scale` now names the codes 4 to 6 of the delayed mode.

```python
    def _analyse_delayed_marginalisation(self, cfg):
        """Delayed marginalisation of the gravity and scale period, every
        column of vio_delayed_marg.csv, when the prior is replaced, what each
        job cost and how long it took to take effect, and what drove it,
        plans/analysis_module.md section 20."""
        figs = []
        dm = {e: df for e, df in self.reader.load("vio_delayed_marg").items() if len(df) > 1}
        if not dm:
            return figs
        configs = self.reader.load_json("config")

        def run_config(exp, key, default):
            c = configs.get(exp, {})
            c = c.get("value0", c)
            return c.get(f"config.{key}", default)

        def col(df, name):
            return df[name].to_numpy() if name in df else np.zeros(len(df))

        events, lags = {}, {}
        for exp, df in dm.items():
            events[exp] = metrics.delayed_marg_events(
                col(df, "started"), col(df, "installed"), col(df, "coarse"),
                col(df, "coarse_phase"), "coarse_spans" in df)
            lags[exp] = metrics.job_install_lag(
                col(df, "started"), col(df, "installed"), col(df, "coarse"),
                bool(run_config(exp, "vio_gs_dm_async", True)))

        def add_multi(key, fig_name, t_ns, quantities, title):
            entry = cfg.get(key, {})
            quantities = {q: runs for q, runs in quantities.items() if runs}
            if not entry.get("enabled") or not t_ns or not quantities:
                return
            figs.append((fig_name, visualisation.multi_axis_time_series(
                t_ns, quantities, title, "t_ns", "value",
                x_range=self.shared_t_range, **self._wh(entry))))

        ec = cfg.get("events", {})
        if ec.get("enabled"):
            t_ev, cat_ev = {}, {}
            for exp, df in dm.items():
                idx, labels = [], []
                for label in metrics.DM_EVENTS:
                    idx.extend(events[exp][label].tolist())
                    labels.extend([label] * len(events[exp][label]))
                if idx:
                    t_ev[exp] = df["t_ns"].to_numpy()[idx]
                    cat_ev[exp] = labels
            if t_ev:
                figs.append(("dm_events", visualisation.categorical_timeline(
                    t_ev, cat_ev, "Prior replacements, readvances and coarse corrections, started, "
                    "installed and discarded, and the start of the delayed phase", "t_ns",
                    x_range=self.shared_t_range, **self._wh(ec))))

        # Rows on which a job finished, installed or discarded.
        fin = {e: np.flatnonzero((col(df, "installed") > 0) | (col(df, "coarse") > 0)) for e, df in dm.items()}
        fin = {e: i for e, i in fin.items() if len(i)}
        period = {e: np.r_[np.nan, np.diff(df["t_ns"].to_numpy()) * 1e-6] for e, df in dm.items()}
        add_multi("job_timing", "dm_job_timing", {e: dm[e]["t_ns"].to_numpy()[i] for e, i in fin.items()}, {
            "job in the worker (ms)": {e: col(dm[e], "job")[i] for e, i in fin.items()},
            "frame period, the budget of a job (ms)": {e: period[e][i] for e, i in fin.items()},
            "VIO thread waiting for the job (ms)": {e: col(dm[e], "wait")[i] for e, i in fin.items() if "wait" in dm[e]},
            "installation and catch up in the VIO thread (ms)": {e: col(dm[e], "catchup")[i] for e, i in fin.items()},
            "steps readvanced at installation": {e: col(dm[e], "catchup_steps")[i] for e, i in fin.items()},
        }, "Cost of each delayed marginalisation job, on the frame it was installed or discarded")
        add_multi("install_lag", "dm_install_lag", {e: dm[e]["t_ns"].to_numpy()[s] for e, (s, _) in lags.items() if len(s)}, {
            "marginalisations from start to installation": {e: l for e, (s, l) in lags.items() if len(s)},
        }, "Delay between starting a job and installing its prior, 1 is the next frame")
        add_multi("store_timing", "dm_store_timing", {e: df["t_ns"] for e, df in dm.items()}, {
            "store capture, push and advance per marginalisation (ms)": {e: col(df, "store") for e, df in dm.items()},
        }, "Per frame cost of keeping the delayed store")

        thresh_s = sorted({float(np.log(run_config(e, "vio_gs_dm_thresh_scale", 1.02))) for e in dm})
        thresh_t = sorted({float(run_config(e, "vio_gs_dm_thresh_tilt_deg", 5.0)) for e in dm})
        add_multi("trigger", "dm_trigger", {e: df["t_ns"] for e, df in dm.items()}, {
            "log scale offset of the block from the prior's anchor, readvance above "
            + ", ".join(f"{v:.4f}" for v in thresh_s): {e: col(df, "anchor_sigma") for e, df in dm.items()},
            "tilt of the block from the prior's anchor (deg), readvance above "
            + ", ".join(f"{v:g}" for v in thresh_t): {e: col(df, "anchor_tilt_deg") for e, df in dm.items()},
        }, "Replacement rule of the delayed phase, the block's offset from the anchor of the prior in use")
        add_multi("store", "dm_store", {e: df["t_ns"] for e, df in dm.items()}, {
            "stored steps": {e: col(df, "steps") for e, df in dm.items()},
            "records of departed variables": {e: col(df, "records") for e, df in dm.items()},
            "coarse phase, 1 coarse and 0 delayed": {e: col(df, "coarse_phase") for e, df in dm.items()},
        }, "Delayed store occupancy and phase")

        cj = {e: np.flatnonzero(col(df, "coarse") > 0) for e, df in dm.items()}
        cj = {e: i for e, i in cj.items() if len(i)}
        coarse_cols = [
            ("coarse_sigma", "estimated log scale"),
            ("coarse_tilt_deg", "tilt of the estimated gravity from the snapshot (deg)"),
            ("coarse_scale_var", "marginal variance of the log scale"),
            ("coarse_scale_free", "scale estimated, 1, or held by the gate, 0"),
            ("coarse_ba", "accelerometer bias norm (m/s^2)"),
            ("coarse_bg", "gyroscope bias norm (rad/s)"),
            ("coarse_cost0", "cost at the snapshot"),
            ("coarse_cost1", "cost at the solution"),
            ("coarse_spans", "keyframe factors"),
            ("coarse_triples", "triples, before Change 16"),
            ("coarse_iterations", "solver iterations"),
            ("coarse_timed_out", "stopped by the deadline, 1"),
            ("coarse_max_vel_inc", "largest velocity increment"),
        ]
        add_multi("coarse_jobs", "dm_coarse_jobs", {e: dm[e]["t_ns"].to_numpy()[i] for e, i in cj.items()}, {
            label: {e: dm[e][c].to_numpy()[i] for e, i in cj.items() if c in dm[e]}
            for c, label in coarse_cols
        }, "Coarse inertial optimisations of the coarse phase, one point per finished job")

        ri = {e: events[e]["readvance installed"] for e in dm}
        ri = {e: i for e, i in ri.items() if len(i)}
        add_multi("smoothing", "dm_smoothing", {e: dm[e]["t_ns"].to_numpy()[i] for e, i in ri.items()}, {
            "largest position increment of the smoothing pass": {e: col(dm[e], "max_pos_inc")[i] for e, i in ri.items()},
        }, "Smoothing of the departed history by each installed readvance")

        gs = self.reader.load("vio_gravity_scale")
        sc = cfg.get("scale_at_events", {})
        if sc.get("enabled"):
            t_gs, y_gs, marks = {}, {}, {}
            for exp, df in gs.items():
                if exp not in dm:
                    continue
                inst = np.r_[events[exp]["readvance installed"], events[exp]["coarse correction installed"]]
                t_inst = dm[exp]["t_ns"].to_numpy()[inst]
                t_gs[exp] = df["t_ns"]
                y_gs[exp] = df["scale_cumulative"]
                marks[exp] = np.flatnonzero(np.isin(df["t_ns"].to_numpy(), t_inst))
            if t_gs:
                figs.append(("dm_scale_at_events", visualisation.time_series(
                    t_gs, y_gs, "Cumulative scale of the correction log, prior replacements marked",
                    "t_ns", "scale", marker_index_per_run=marks,
                    x_range=self.shared_t_range, **self._wh(sc))))

        hc = cfg.get("job_duration_histogram", {})
        jobs = {e: col(dm[e], "job")[i] for e, i in fin.items()}
        jobs = {e: j[j > 0] for e, j in jobs.items() if (j > 0).any()}
        if hc.get("enabled") and jobs:
            figs.append(("dm_job_duration_histogram", visualisation.histogram(
                jobs, "Duration of delayed marginalisation jobs in the worker", "job (ms)",
                bins=hc.get("bins", 30), **self._wh(hc))))

        if not cfg.get("scalars", {}).get("enabled"):
            return figs
        lift = self.reader.liftoff_t_ns
        for exp, df in dm.items():
            add = lambda name, value, type_="value": self._add_metric("delayed_marginalisation", exp, name, type_, value)
            t = df["t_ns"].to_numpy()
            for label in metrics.DM_EVENTS[:5]:
                add(f"Events, {label}", int(len(events[exp][label])))
            begin = events[exp]["delayed phase begins"]
            if len(begin):
                add("Delayed phase begins (s)", t[begin[0]] * 1e-9)
                if exp in lift:
                    add("Delayed phase begins after lift off (s)", (t[begin[0]] - lift[exp]) * 1e-9)
                span_min = (t[-1] - t[begin[0]]) * 1e-9 / 60.0
                n = int((events[exp]["readvance installed"] >= begin[0]).sum())
                if span_min > 0:
                    add("Readvances per minute in the delayed phase", n / span_min, "rate")
            if exp in jobs:
                j = jobs[exp]
                add("Job duration (ms)", float(np.median(j)), "median")
                add("Job duration (ms)", float(np.percentile(j, 95)), "percentile")
                add("Job duration (ms)", float(j.max()), "max")
                budget = period[exp][fin[exp]]
                add("Jobs longer than the frame period", int(np.sum(col(df, "job")[fin[exp]] > budget)))
            if exp in fin:
                add("Installation and catch up (ms)", float(col(df, "catchup")[fin[exp]].max()), "max")
            if "wait" in df:
                w = col(df, "wait")
                add("VIO thread wait for a job (ms)", float(w.max()), "max")
                add("Frames on which the VIO thread waited", int((w > 0).sum()))
            add("Store cost per marginalisation (ms)", float(np.median(col(df, "store"))), "median")
            add("Store cost per marginalisation (ms)", float(col(df, "store").max()), "max")
            s, l = lags[exp]
            if len(l):
                add("Marginalisations from start to installation", float(l.mean()), "mean")
                add("Marginalisations from start to installation", int(l.max()), "max")
            if "coarse_timed_out" in df:
                add("Coarse jobs stopped by the deadline", int(col(df, "coarse_timed_out").sum()))
            if exp in ri:
                add("Largest smoothing position increment", float(col(df, "max_pos_inc")[ri[exp]].max()), "max")
            add("Most stored steps", int(col(df, "steps").max()), "max")
            add("Most records", int(col(df, "records").max()), "max")
        return figs
```

#### Change 3. `config.json`

```json
  "delayed_marginalisation": {
    "events": {"enabled": true, "width": 1080, "height": 400},
    "job_timing": {"enabled": true, "width": 1080, "height": 1250},
    "install_lag": {"enabled": true, "width": 1080, "height": 400},
    "store_timing": {"enabled": true, "width": 1080, "height": 400},
    "trigger": {"enabled": true, "width": 1080, "height": 750},
    "store": {"enabled": true, "width": 1080, "height": 750},
    "coarse_jobs": {"enabled": true, "width": 1080, "height": 2600},
    "smoothing": {"enabled": true, "width": 1080, "height": 400},
    "scale_at_events": {"enabled": true, "width": 1080, "height": 400},
    "job_duration_histogram": {"enabled": true, "bins": 30, "width": 720, "height": 540},
    "scalars": {"enabled": true}
  }
```

#### Change 4. `README.md`

The tab list of the `config.json` section gains `delayed_marginalisation`, and a bullet describes its figures and scalars.

### Backwards compatibility impact

No existing figure or scalar changes, except the title of `gs_swap_events`. The tab adds figures to `figures/delayed_marginalisation/` and rows to `metrics.csv` under one new category, and returns no figure for a run without `vio_delayed_marg.csv`, so logs of runs without the period are unaffected. Both schemas of the CSV are read.

### Validation strategy

1. In the dev container, `python3 -m py_compile` of `analysis.py` and `metrics.py`, and `config.json` parses. Passed 2026-11-14.
2. In the dev container, `metrics.delayed_marg_events` and `metrics.job_install_lag` on `/ws/logs/slam/2026-11-13_22-33-36`, run 2 of `plans/delayed_marginalisation.md` §5. They found 12 coarse corrections, 37 readvances, the delayed phase beginning 10.53 s after the first row and 5.26 s after lift off, and every one of the 49 jobs installed one marginalisation after its start, with job durations of 28.1 ms at the median and 35.6 ms at most, which agree with §5.1 of that plan.
3. In the dev container, `Analysis._analyse_delayed_marginalisation` executed with pandas, Bokeh, Plotly and SciPy replaced by stubs, a minimal stand-in for `DataFrame` over the same run, and the four `visualisation` functions it calls recorded rather than drawn. All ten figures were built with every trace's length equal to its time axis, and the scalars above were written. The same check on a copy of that run rewritten to the schema of Change 17, with one coarse job marked discarded by its deadline, drew the wait and the six newer coarse columns and counted the discarded job.
4. Outstanding, in `analysis_container`, `djinn start slam-analysis` on a directory with a run of each schema, confirming the tab renders and its zoom links with the other tabs.

### Deliberately not done

No change to `raw_logs`, which still stacks every column of `vio_delayed_marg.csv` as well. No log scale axis for the scale variance, since `multi_axis_time_series` has no per subplot axis type and the variance spans orders of magnitude only before lift off. No threshold traces in `dm_trigger`, since a trace per run would need colours outside the seeded run palette, which `visualisation.seed_run_colours` assigns before any pooled work, so the thresholds are written into the subplot titles.

## References

`[1]` through `[25]` are as numbered in `context/slam_analysis.md`'s own References section, cited here unchanged wherever this plan restates a quantity that document already grounds.

`[26]` Plotly documentation, legend interactivity, "users may show or hide traces by clicking or double-clicking on their associated legend item," https://plotly.com/python/legend/.

`[27]` HoloViz Panel documentation, `Tabulator` widget reference, client side header filters, https://panel.holoviz.org/reference/widgets/Tabulator.html.

`[28]` HoloViz Panel documentation, "Saving and Embedding" and "Embedding and Exporting Panel Apps," the `resources=INLINE` static export path and the `embed=True` combinatorial state warning, https://panel.holoviz.org/how_to/export/saving.html, https://panel.holoviz.org/how_to/export/embedding.html.

`[29]` Bokeh documentation, "Embedding Bokeh Content," `bokeh.embed.file_html`, https://docs.bokeh.org/en/latest/docs/user_guide/output/embed.html.

`[30]` Bokeh documentation and wiki, `CustomJSFilter`/`CDSView`, https://docs.bokeh.org/en/latest/docs/reference/models/filters.html, https://github.com/bokeh/bokeh/wiki/Filterable-Data-Source.

`[31]` `mwouts/itables` repository, `src/itables/javascript.py`, `to_html_datatable`'s offline bundled template, https://github.com/mwouts/itables.

`[32]` PyPI JSON metadata, dependency counts for `panel`, `bokeh`, `plotly` and `itables`, https://pypi.org/pypi/panel/json, https://pypi.org/pypi/bokeh/json, https://pypi.org/pypi/plotly/json, https://pypi.org/pypi/itables/json.

`[33]` Datapane, archived in substance, last PyPI release pinning `pandas<2.0`/`plotly<6.0` against current versions, https://pypi.org/project/datapane/, https://api.github.com/repos/datapane/datapane.
