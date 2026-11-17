import logging
import os
import time
from concurrent.futures import ProcessPoolExecutor
from contextlib import nullcontext

import metrics
import numpy as np
import pandas as pd
from bokeh.io import save as bokeh_save
from bokeh.models import DataRange1d, LayoutDOM
from bokeh.resources import INLINE
from data_reader import DataReader
from scipy.spatial.transform import Rotation
from trajectory_analysis import TrajectoryAnalysis

import visualisation

logger = logging.getLogger(__name__)


class _Immediate:
    """Wraps an already-computed value behind a Future-shaped .result(), so
    the three merge loops below read self._pool.submit(...) and the direct
    sequential call through one identical interface rather than branching
    on self._pool at every merge site."""

    __slots__ = ("_value",)

    def __init__(self, value):
        self._value = value

    def result(self):
        return self._value


def _trajectory_worker(exp, t_ns, gt_xyz, gt_q, est_xyz, est_q, alignment_mode, cfg, yaw_offset):
    """Per-experiment body of _analyse_trajectory's own loop, module level
    and picklable so a ProcessPoolExecutor worker can run it: plain numpy
    arrays and a plain JSON-sourced cfg sub-dict in, a plain dict out. Builds
    its own TrajectoryAnalysis so no alignment state is shared across
    experiments or across processes."""
    ta = TrajectoryAnalysis(alignment_mode)
    ta.set_alignment(gt_xyz, gt_q, est_xyz, est_q)
    est_xyz_aligned, est_q_aligned = ta.apply(est_xyz, est_q)
    out = {
        "est_xyz_aligned": est_xyz_aligned,
        "ate": None,
        "err_xyz": None,
        "rpe_rmse_curve": None,
        "rpe_rmse_per_delta_series": None,
        "rpe_length_per_delta_series": None,
        "yaw_offset": yaw_offset,
        "drift_proxies": None,
    }

    if cfg.get("ate", {}).get("enabled"):
        out["ate"] = ta.ate(gt_xyz, est_xyz_aligned)

    if cfg.get("trajectory_error_xyz", {}).get("enabled"):
        out["err_xyz"] = est_xyz_aligned - gt_xyz

    if cfg.get("rpe_rmse", {}).get("enabled"):
        curve, per_delta_series = ta.rpe(
            gt_xyz, gt_q, est_xyz_aligned, est_q_aligned, cfg["rpe_rmse"]["deltas"]
        )
        out["rpe_rmse_curve"] = curve
        out["rpe_rmse_per_delta_series"] = per_delta_series

    if cfg.get("rpe_length", {}).get("enabled"):
        lengths = cfg["rpe_length"]["length"]
        lengths = lengths if isinstance(lengths, list) else [lengths]
        _, per_delta_series = ta.rpe(gt_xyz, gt_q, est_xyz_aligned, est_q_aligned, lengths)
        out["rpe_length_per_delta_series"] = per_delta_series

    if cfg.get("drift_proxies", {}).get("enabled"):
        out["drift_proxies"] = metrics.drift_proxies(gt_xyz, gt_q, est_xyz, est_q)

    ta.reset_alignment()
    return out


def _excitation_worker(imu_df, st_df, g, window_s):
    """Per-experiment body of _analyse_vio_optimisation's own
    excitation_covariance loop, module level and picklable for the same
    reason as _trajectory_worker. Returns None on an empty merge, exactly
    the "continue" the original loop took in that case."""
    merged = metrics.align_frames_on_t_ns(
        imu_df[["t_ns", "accel_mean_0", "accel_mean_1", "accel_mean_2"]],
        st_df[["t_ns", "q_w_i_0", "q_w_i_1", "q_w_i_2", "q_w_i_3"]],
    )
    if merged.empty:
        return None
    accel_mean_body = DataReader.vec(merged, "accel_mean", 3)
    q = DataReader.vec(merged, "q_w_i", 4)
    R = Rotation.from_quat(q).as_matrix()
    accel_mean_world = np.einsum("nij,nj->ni", R, accel_mean_body) - g
    dt_s = float(np.median(np.diff(merged["t_ns"].to_numpy()))) * 1e-9
    cov_min_ev = metrics.excitation_covariance(accel_mean_world, window_s, dt_s)
    return merged["t_ns"], cov_min_ev

TABS = [
    "trajectory",
    "optical_flow_detection",
    "optical_flow_tracking",
    "vio_triangulation",
    "vio_optimisation",
    "gravity_scale",
    "delayed_marginalisation",
    "stationary_detection",
    "mapping_detection",
    "mapping_matching",
    "mapping_triangulation",
    "mapping_optimisation",
    "timing",
    "raw_logs",
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
    def __init__(self, reader: DataReader, config: dict, out_dir: str, workers: int | None = None):
        self.reader, self.config, self.out_dir = reader, config, out_dir
        self.workers = workers if workers is not None else reader.workers
        self.metrics_rows: list[dict] = []
        self.tabs: dict[str, list[tuple[str, object]]] = {}
        # Live only for the duration of run()'s own tab loop, None otherwise
        # and whenever self.workers <= 1, so every dispatch site below can
        # test "self._pool is None" rather than threading a separate flag.
        self._pool = None
        # Precomputed once, deterministically, from the already-sorted
        # experiment_ids, before run() can start any pooled work; see
        # visualisation.seed_run_colours's own docstring for why lazy,
        # first-touch colour assignment is a concurrency hazard otherwise.
        visualisation.seed_run_colours(self.reader.experiment_ids)
        # Shared across every call site below that passes it as x_range,
        # Bokeh's own linked panning, a figure whose x_range is this same
        # DataRange1d instance pans/zooms together with every other one that
        # shares it, including figures on other tabs. DataRange1d, not
        # Range1d, so it auto-fits to the union of every renderer that uses
        # it, Range1d is a fixed, manually bounded range that never auto
        # fits, left at its default (start=None, end=None) it renders
        # degenerate and empty. One instance per Analysis instance, which
        # under dashboard.py's panel serve model means one per browser
        # session, never shared across two sessions' own Bokeh documents.
        self.shared_t_range = DataRange1d()

    def _write_figure(self, fig, path: str, title: str) -> None:
        if isinstance(fig, LayoutDOM):
            bokeh_save(fig, filename=path, resources=INLINE, title=title)
        else:
            fig.write_html(path, include_plotlyjs=True)

    def run(self):
        t_run = time.perf_counter()
        logger.info("Analysis.run() starting, %d experiments under %s",
                    len(self.reader.experiment_ids), self.out_dir)
        # Created fresh here, never at import time or in a constructor, so
        # no CuPy array can exist in the parent process before fork (Linux's
        # default multiprocessing start method) duplicates it. If cupy is
        # importable (true on analysis_container with --gpus passed, absent
        # from the dev container), metrics.relative_pose_error_sweep still
        # defaults to the GPU inside _trajectory_worker with no override
        # here, so --workers > 1 on such a host means every pooled worker
        # process independently acquires its own CUDA context, which can
        # contend for a single GPU; see plans/analysis_module.md section 18.
        pool_ctx = ProcessPoolExecutor(max_workers=self.workers) if self.workers > 1 else nullcontext()
        with pool_ctx as pool:
            self._pool = pool
            for tab in TABS:
                method = getattr(self, f"_analyse_{tab}")
                t_tab = time.perf_counter()
                self.tabs[tab] = method(self.config.get(tab, {}))
                logger.info("  _analyse_%s: %.2fs, %d figures", tab,
                            time.perf_counter() - t_tab, len(self.tabs[tab]))
        self._pool = None
        os.makedirs(os.path.join(self.out_dir, "figures"), exist_ok=True)
        t_write = time.perf_counter()
        for tab, figs in self.tabs.items():
            tab_dir = os.path.join(self.out_dir, "figures", tab)
            os.makedirs(tab_dir, exist_ok=True)
            for title, fig in figs:
                self._write_figure(fig, os.path.join(tab_dir, f"{title}.html"), title)
        logger.info("  figures/ written: %.2fs", time.perf_counter() - t_write)
        metrics.pivot_metrics(self.metrics_rows).to_csv(
            os.path.join(self.out_dir, "metrics.csv"), index=False
        )
        logger.info("Analysis.run() total: %.2fs", time.perf_counter() - t_run)

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
        # ta = TrajectoryAnalysis(cfg.get("alignment_mode", "posyaw"))  # superseded 2026-10-26: each
        # _trajectory_worker call now builds its own instance, so alignment state never crosses a
        # process (or even an experiment) boundary; see plans/analysis_module.md section 18.
        t_ns_per_run, est_xyz_per_run, err_per_run, gt_xyz_per_run = {}, {}, {}, {}
        err_xyz_per_run = {}
        rpe_rmse_series, rpe_length_series = {}, {}

        alignment_mode = cfg.get("alignment_mode", "posyaw")
        futures = {}
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
            t_ns_per_run[exp] = t_ns
            gt_xyz_per_run[exp] = gt_xyz
            yaw_offset = (
                vio_init[exp]["yaw_offset_rad"].iloc[0]
                if cfg.get("yaw_offset", {}).get("enabled") and exp in vio_init
                else None
            )
            args = (exp, t_ns, gt_xyz, gt_q, est_xyz, est_q, alignment_mode, cfg, yaw_offset)
            futures[exp] = (
                self._pool.submit(_trajectory_worker, *args)
                if self._pool
                else _Immediate(_trajectory_worker(*args))
            )

        for exp in self.reader.experiment_ids:
            if exp not in futures:
                continue
            result = futures[exp].result()
            est_xyz_per_run[exp] = result["est_xyz_aligned"]

            if result["ate"] is not None:
                for stat in ("rmse", "mean", "median", "std", "max"):
                    self._add_metric("trajectory", exp, "ATE (m)", stat, result["ate"][stat])
                err_per_run[exp] = result["ate"]["residual"]

            if result["err_xyz"] is not None:
                err_xyz_per_run[exp] = result["err_xyz"]

            if result["rpe_rmse_curve"] is not None:
                for delta, rmse in result["rpe_rmse_curve"]:
                    self._add_metric(
                        "trajectory", exp, f"RPE RMSE, {delta} frames (m)", "rmse", rmse
                    )
                rpe_rmse_series[exp] = result["rpe_rmse_per_delta_series"]

            if result["rpe_length_per_delta_series"] is not None:
                rpe_length_series[exp] = (t_ns_per_run[exp], result["rpe_length_per_delta_series"])

            if result["yaw_offset"] is not None:
                self._add_metric("trajectory", exp, "Yaw offset (rad)", "value", result["yaw_offset"])

            if result["drift_proxies"] is not None:
                for key, value in result["drift_proxies"].items():
                    name, type_ = DRIFT_PROXY_LABELS[key]
                    self._add_metric("trajectory", exp, name, type_, value)

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
                                x_range=self.shared_t_range,
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
                                x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                                x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
                        **self._wh(cfg["marg_size"]),
                    ),
                )
            )

        if cfg.get("marg_eigenspectrum", {}).get("enabled") and vio_marg_nullspace:
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
                            x_range=self.shared_t_range,
                            **self._wh(cfg["marg_eigenspectrum"]),
                        ),
                    )
                )

        if cfg.get("marg_nullspace_response", {}).get("enabled") and vio_marg_nullspace:
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
                        x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
                            **self._wh(cfg["state_trace"]),
                        ),
                    )
                )

        if cfg.get("excitation_covariance", {}).get("enabled"):
            window_s = cfg["excitation_covariance"].get("window_s", 1.5)
            t_ns, y = {}, {}
            # Joined on t_ns inside _excitation_worker, rather than truncated
            # to min(len(imu_df), len(st_df)) and zipped by row position,
            # because the IMU frame record and the state trace are two
            # independent logging sites and a frame dropped by only one of
            # them shifts their row counts out of step, which a positional
            # zip would silently pair against the wrong frame.
            futures = {}
            for exp, imu_df in vio_imu_frame.items():
                if exp not in vio_state or exp not in vio_init:
                    continue
                g = self.reader.vec(vio_init[exp].iloc[[0]], "g", 3)[0]
                args = (imu_df, vio_state[exp], g, window_s)
                futures[exp] = (
                    self._pool.submit(_excitation_worker, *args)
                    if self._pool
                    else _Immediate(_excitation_worker(*args))
                )
            for exp in self.reader.experiment_ids:
                if exp not in futures:
                    continue
                result = futures[exp].result()
                if result is None:
                    continue
                t_ns[exp], y[exp] = result
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
                            x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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

    def _analyse_gravity_scale(self, cfg):
        """Gravity and scale period of the VIO estimator, every column of
        vio_gravity_scale.csv and the physical scale and gravity errors
        against ground truth, plans/analysis_module.md section 19."""
        figs = []
        gs = self.reader.load("vio_gravity_scale")
        lift = self.reader.liftoff_t_ns
        upd = {
            exp: metrics.block_update_mask(df["sigma_step"].to_numpy(), df["tilt_step_deg"].to_numpy())
            for exp, df in gs.items()
        }

        def add_multi(key, fig_name, t_ns, quantities, title):
            entry = cfg.get(key, {})
            quantities = {q: runs for q, runs in quantities.items() if runs}
            if not entry.get("enabled") or not t_ns or not quantities:
                return
            figs.append((fig_name, visualisation.multi_axis_time_series(
                t_ns, quantities, title, "t_ns", "value",
                x_range=self.shared_t_range, **self._wh(entry))))

        if gs:
            t_upd = {exp: df["t_ns"][upd[exp]] for exp, df in gs.items()}
            add_multi("block_step", "gs_block_step", t_upd, {
                "log scale step, sigma_step": {e: df["sigma_step"][upd[e]] for e, df in gs.items()},
                "tilt step (deg)": {e: df["tilt_step_deg"][upd[e]] for e, df in gs.items()},
                "random walk cost of the step, jump_cost": {e: df["jump_cost"][upd[e]] for e, df in gs.items()},
            }, "Gravity and scale block, step applied on each update frame")
            t_all = {exp: df["t_ns"] for exp, df in gs.items()}
            add_multi("block_cumulative", "gs_block_cumulative", t_all, {
                "cumulative scale of the correction log": {e: df["scale_cumulative"] for e, df in gs.items()},
                "cumulative rotation of the correction log (deg)": {e: df["tilt_cumulative_deg"] for e, df in gs.items()},
                "correction log epoch": {e: df["epoch"] for e, df in gs.items()},
            }, "Gravity and scale block, cumulative correction applied to every state")
            add_multi("visual_prior_admission", "gs_visual_prior_admission", t_all, {
                "inertial factors admitted in full to M_visual": {e: df["visual_imu_full"] for e, df in gs.items()},
                "inertial factors admitted as rotation rows only": {e: df["visual_imu_rot"] for e, df in gs.items()},
                "largest whitened scale column of a departing factor": {e: df["visual_imu_scale_col"] for e, df in gs.items()},
            }, "Change A admission of departing inertial factors into M_visual")
            sc = cfg.get("swap_events", {})
            swaps = {exp: df[df["swap_event"] > 0] for exp, df in gs.items()}
            swaps = {exp: df for exp, df in swaps.items() if not df.empty}
            if sc.get("enabled") and swaps:
                figs.append(("gs_swap_events", visualisation.categorical_timeline(
                    {e: df["t_ns"] for e, df in swaps.items()},
                    {e: df["swap_event"].astype(int) for e, df in swaps.items()},
                    "Prior swaps, 1 half reset, 2 upward crossing, 3 downward crossing (dynamic "
                    "marginalisation), 4 fold, 5 coarse correction, 6 readvance (delayed marginalisation)",
                    "t_ns", x_range=self.shared_t_range, **self._wh(sc))))

        gt = self.reader.load("gt_eval")
        st = self.reader.load("vio_state")
        pc = cfg.get("physical_scale_gravity", {})
        physical = {}
        for exp, df in gt.items():
            df = df.dropna(subset=[f"p_gt_{i}" for i in range(3)] + [f"p_w_i_{i}" for i in range(3)])
            if len(df) < 3:
                continue
            t = df["t_ns"].to_numpy()
            p_est, p_gt = DataReader.vec(df, "p_w_i", 3), DataReader.vec(df, "p_gt", 3)
            q_est, q_gt = DataReader.vec(df, "q_w_i", 4), DataReader.vec(df, "q_gt", 4)
            out = {
                "t_ns": t,
                "ratio": metrics.windowed_scale_ratio(t, p_est, p_gt, pc.get("window_s", 2.0), pc.get("min_disp_m", 0.3)),
                "tilt": metrics.tilt_error_deg(q_est, q_gt),
                "yaw": metrics.yaw_error_deg(q_est, q_gt),
                "speed": np.full(len(t), np.nan),
            }
            if exp in st:
                v = st[exp][["t_ns"] + [f"v_w_i_{i}" for i in range(3)]]
                m = metrics.align_frames_on_t_ns(df[["t_ns"] + [f"p_gt_{i}" for i in range(3)]], v)
                if len(m) >= 3:
                    v_gt = metrics.finite_difference_velocity(m["t_ns"].to_numpy(), DataReader.vec(m, "p_gt", 3))
                    s_gt = np.linalg.norm(v_gt, axis=1)
                    s_est = np.linalg.norm(DataReader.vec(m, "v_w_i", 3), axis=1)
                    ratio = np.where(s_gt > pc.get("min_speed_mps", 0.3), s_est / np.maximum(s_gt, 1e-9), np.nan)
                    out["speed"] = np.interp(t, m["t_ns"].to_numpy(), ratio, left=np.nan, right=np.nan)
            physical[exp] = out
        add_multi("physical_scale_gravity", "gs_physical_scale_gravity",
                  {e: o["t_ns"] for e, o in physical.items()}, {
                      "scale ratio, |estimated| / |true| displacement over the window": {e: o["ratio"] for e, o in physical.items()},
                      "tilt error, body frame gravity (deg)": {e: o["tilt"] for e, o in physical.items()},
                      "yaw error (deg)": {e: o["yaw"] for e, o in physical.items()},
                      "speed ratio, |estimated| / |true| velocity": {e: o["speed"] for e, o in physical.items()},
                  }, "Physical scale and gravity errors against ground truth")

        if st:
            ba = {e: DataReader.vec(df, "ba", 3) for e, df in st.items()}
            add_multi("bias_components", "gs_bias_components", {e: df["t_ns"] for e, df in st.items()}, {
                **{f"accelerometer bias {a} (m/s^2)": {e: b[:, i] for e, b in ba.items()} for i, a in enumerate("xyz")},
                "tilt equivalent of the horizontal accelerometer bias (deg)": {e: metrics.bias_equivalent_tilt_deg(b) for e, b in ba.items()},
                **{f"gyroscope bias {a} (rad/s)": {e: df[f"bg_{i}"] for e, df in st.items()} for i, a in enumerate("xyz")},
            }, "Bias components, body frame")

        jumps = {}
        for exp, df in st.items():
            t = df["t_ns"].to_numpy()
            jumps[exp] = metrics.position_jump(t, DataReader.vec(df, "p_w_i", 3), DataReader.vec(df, "v_w_i", 3))
        jc = cfg.get("position_jump", {})
        if jc.get("enabled") and jumps:
            markers = {
                exp: np.flatnonzero(np.isin(st[exp]["t_ns"].to_numpy(), gs[exp]["t_ns"].to_numpy()[upd[exp]]))
                for exp in jumps if exp in gs
            }
            figs.append(("gs_position_jump", visualisation.time_series(
                {e: st[e]["t_ns"] for e in jumps}, jumps,
                "Per frame position jump not explained by velocity, update frames marked",
                "t_ns", "jump (m)", marker_index_per_run=markers,
                x_range=self.shared_t_range, **self._wh(jc))))

        pu = self.reader.load("vio_pose_update")
        add_multi("pose_update_gate", "gs_pose_update_gate", {e: df["t_ns"] for e, df in pu.items()}, {
            c: {e: df[c] for e, df in pu.items() if c in df}
            for c in ("applied", "rejected", "max_trans_err", "max_rot_err")
        }, "Mapper pose updates at the estimator's gate, limits limit_trans and limit_rot")
        ss = self.reader.load("vio_solver_summary")
        add_multi("solver_health", "gs_solver_health", {e: df["t_ns"] for e, df in ss.items()}, {
            c: {e: df[c] for e, df in ss.items() if c in df}
            for c in ("num_it", "num_it_rejected", "converged")
        }, "VIO solver iterations, rejected iterations and convergence per frame")

        sc = cfg.get("scalars", {})
        if not sc.get("enabled"):
            return figs
        lo, hi = sc.get("median_ratio_window_s", [5, 30])
        tilt_at = sc.get("tilt_at_s", 10)
        for exp in self.reader.experiment_ids:
            add = lambda name, value, type_="value": self._add_metric("gravity_scale", exp, name, type_, value)
            t_lift = lift.get(exp)
            if t_lift is not None:
                add("Lift off time (s)", t_lift * 1e-9)
            if exp in gs:
                df, u = gs[exp], upd[exp]
                tg = df["t_ns"].to_numpy()
                swap = df["swap_event"].to_numpy() > 0
                span_s = (tg[-1] - tg[0]) * 1e-9 if len(tg) > 1 else np.nan
                add("Block updates per second", u.sum() / span_s if span_s else np.nan, "rate")
                add("Swaps", int(swap.sum()))
                add("Swaps on held frames", int((swap & ~u).sum()))
                add("Largest log scale step", float(np.abs(df["sigma_step"]).max()), "max")
                add("Largest tilt step (deg)", float(np.abs(df["tilt_step_deg"][u]).max()) if u.any() else 0.0, "max")
                add("Final cumulative scale", float(df["scale_cumulative"].iloc[-1]))
                add("Largest cumulative rotation (deg)", float(np.abs(df["tilt_cumulative_deg"]).max()), "max")
                if t_lift is not None:
                    after = u & (tg >= t_lift)
                    if after.any():
                        add("First block update after lift off (s)", (tg[after][0] - t_lift) * 1e-9)
            if exp in physical and t_lift is not None:
                o = physical[exp]
                rel = (o["t_ns"] - t_lift) * 1e-9
                post = rel >= 0
                if post.any():
                    add("Largest tilt error after lift off (deg)", float(np.nanmax(o["tilt"][post])), "max")
                    i = np.searchsorted(rel, tilt_at)
                    if i < len(rel):
                        add(f"Tilt error {tilt_at} s after lift off (deg)", float(o["tilt"][i]))
                    w = (rel >= lo) & (rel <= hi)
                    if np.isfinite(o["ratio"][w]).any():
                        add(f"Scale ratio {lo} s to {hi} s after lift off", float(np.nanmedian(o["ratio"][w])), "median")
            if exp in jumps and exp in gs:
                on_upd = np.isin(st[exp]["t_ns"].to_numpy(), gs[exp]["t_ns"].to_numpy()[upd[exp]])
                if on_upd.any():
                    add("Largest position jump on update frames (m)", float(jumps[exp][on_upd].max()), "max")
                if (~on_upd).any():
                    add("Largest position jump on held frames (m)", float(jumps[exp][~on_upd].max()), "max")
            if exp in pu:
                total = pu[exp]["applied"].sum() + pu[exp]["rejected"].sum()
                if total:
                    add("Rejected fraction of pose updates", float(pu[exp]["rejected"].sum() / total), "rate")
        return figs

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
            "fold test satisfied": {e: col(df, "fold_settled") for e, df in dm.items() if "fold_settled" in df},
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

    def _analyse_stationary_detection(self, cfg):
        figs = []
        stationary = self.reader.load("vio_stationary")

        if cfg.get("metrics", {}).get("enabled") and stationary:
            mc = cfg["metrics"]
            t_ns = {exp: df["t_ns"] for exp, df in stationary.items()}
            quantities = {
                "SHOE statistic, t_shoe": {},
                "Mahalanobis distance squared, d2": {},
                "optical flow magnitude (px)": {},
            }
            for exp, df in stationary.items():
                quantities["SHOE statistic, t_shoe"][exp] = df["t_shoe"]
                quantities["Mahalanobis distance squared, d2"][exp] = df["d2"]
                quantities["optical flow magnitude (px)"][exp] = df["flow_mean"]
            figs.append(
                (
                    "stationary_detector_metrics",
                    visualisation.multi_axis_time_series(
                        t_ns,
                        quantities,
                        "Stationary detector metrics, one test statistic per stage",
                        "t_ns",
                        "value",
                        x_range=self.shared_t_range,
                        **self._wh(mc),
                    ),
                )
            )

        if cfg.get("stage_flags", {}).get("enabled") and stationary:
            fc = cfg["stage_flags"]
            t_ns = {exp: df["t_ns"] for exp, df in stationary.items()}
            for col, label in (
                ("stage1", "Stage 1, windowed SHOE"),
                ("stage2", "Stage 2, Mahalanobis"),
                ("stage3", "Stage 3, optical flow veto"),
                ("is_stationary", "Combined mpIsStationary"),
            ):
                flag = {exp: df[col] for exp, df in stationary.items()}
                figs.append(
                    (
                        f"stationary_{col}",
                        visualisation.categorical_timeline(
                            t_ns,
                            flag,
                            f"{label}, 1 clears, 0 does not",
                            "t_ns",
                            x_range=self.shared_t_range,
                            **self._wh(fc),
                        ),
                    )
                )

        if cfg.get("vehicle_speed", {}).get("enabled"):
            vc = cfg["vehicle_speed"]
            vio_state = self.reader.load("vio_state")
            t_ns = {exp: df["t_ns"] for exp, df in vio_state.items()}
            y = {exp: df["v_norm"] for exp, df in vio_state.items()}
            figs.append(
                (
                    "vehicle_speed",
                    visualisation.time_series(
                        t_ns,
                        y,
                        "VISLAM estimated vehicle speed",
                        "t_ns",
                        "v_norm (m/s)",
                        x_range=self.shared_t_range,
                        **self._wh(vc),
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
                        x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
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
            futures = {}
            for exp, df in bow_candidate.items():
                if exp not in gt_eval:
                    continue
                args = (df, gt_eval[exp], pr_cfg["distance_thresh_m"], pr_cfg["min_time_gap_s"])
                futures[exp] = (
                    self._pool.submit(metrics.bow_pr_curve, *args)
                    if self._pool
                    else _Immediate(metrics.bow_pr_curve(*args))
                )
            for exp in self.reader.experiment_ids:
                if exp not in futures:
                    continue
                recall, precision, is_true = futures[exp].result()
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
                            x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                        x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
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
                            x_range=self.shared_t_range,
                            **self._wh(fc),
                        ),
                    )
                )
        return figs

    def _analyse_raw_logs(self, cfg):
        """Every non constant numeric column of every CSV found in any
        experiment, one stacked figure per CSV, so nothing logged stays
        invisible. Columns constant within every run become scalars,
        plans/analysis_module.md section 19."""
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
                            self._add_metric("raw_logs", e, f"{name}.{c}", "value",
                                             float(d[c].dropna().iloc[0]))
                else:
                    plotted.append(c)
            if not plotted:
                continue
            step = {e: max(1, len(d) // max_points) for e, d in data.items()}
            t_ns = {e: d["t_ns"].iloc[::step[e]] for e, d in data.items()}
            quantities = {c: {e: d[c].iloc[::step[e]] for e, d in data.items() if c in d}
                          for c in plotted}
            figs.append((f"raw_{name}", visualisation.multi_axis_time_series(
                t_ns, quantities, f"{name}.csv, every non constant numeric column", "t_ns", "value",
                x_range=self.shared_t_range, width=cfg.get("width", 1080),
                height=min(per_plot * len(plotted), 6000))))
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
