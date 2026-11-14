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
    """Every experiment replays the same mission, DataReader._scan_epochs has
    already offset every one of its logs, this one included, by that
    experiment's own mission start, so ground truth positions agree within a
    few metres at matching mission relative times across every experiment
    (confirmed directly against /ws/logs/slam/, every phase_* and dated run
    sampled within a few metres of each other at t=30/50/70/90/120/150s).
    The samples themselves are not frame synchronised though, independently
    logged runs do not tick on the same nanosecond grid, so concatenating
    every experiment's own ground truth and deduplicating on an EXACT t_ns
    match, this function's own earlier approach, almost never actually drops
    anything, since an exact nanosecond collision between two independent
    runs is rare, and renders as several near identical ground truth strands
    braided together rather than one clean reference curve. Every experiment
    here already covers the full mission with near complete, near identical
    coverage (no gaps one needs another experiment's own samples to fill),
    so the one combined reference this returns is simply the single
    experiment with the most logged ground truth samples, picked
    deterministically rather than attempting a fuzzy, proximity based merge
    this data does not actually need."""
    best_exp = max(t_ns_per_run, key=lambda exp: len(t_ns_per_run[exp]))
    return np.asarray(t_ns_per_run[best_exp]), gt_xyz_per_run[best_exp]


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


# Gravity and scale period, plans/analysis_module.md section 19. NumPy only, so
# these can be exercised without pandas or SciPy. Quaternions are (x, y, z, w),
# the order Logger::AddGtEval and Logger::AddVioState write (Eigen coeffs()).
G_NORM = 9.81


def _rotate(q_xyzw: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotates each row of v by the matching unit quaternion."""
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
    """Heading difference of the body x axis on the horizontal plane."""
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


def finite_difference_velocity(t_ns: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    return np.gradient(xyz, t_ns * 1e-9, axis=0)


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
