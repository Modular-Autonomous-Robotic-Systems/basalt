import numpy as np
from scipy.spatial.transform import Rotation

from alignment import Alignment
from metrics import relative_pose_error_sweep


def poses_to_se3(p_xyz: np.ndarray, q_xyzw: np.ndarray) -> np.ndarray:
    """(N, 3) and (N, 4) x,y,z,w quaternions to (N, 4, 4) SE(3). Matches
    scipy's own from_quat order, no reordering needed."""
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
        """Computes and stores this experiment's alignment once. gt_qxyzw/
        est_qxyzw are accepted for a signature uniform with ate()/rpe() even
        though every alignment mode here is position only."""
        self.s, self.R, self.t = self.ALIGN[self.mode](gt_xyz, est_xyz)

    def reset_alignment(self) -> None:
        self.R, self.t, self.s = None, None, 1.0

    def apply(self, est_xyz: np.ndarray, est_qxyzw: np.ndarray):
        """Call once per experiment; ate()/rpe() take the result rather than
        re-deriving it."""
        if self.R is None:
            raise RuntimeError("TrajectoryAnalysis.set_alignment() must be called before ate()/rpe()/apply()")
        est_xyz_aligned = (self.s * (self.R @ est_xyz.T)).T + self.t
        est_q_aligned = (Rotation.from_matrix(self.R) * Rotation.from_quat(est_qxyzw)).as_quat()
        return est_xyz_aligned, est_q_aligned

    def ate(self, gt_xyz: np.ndarray, est_xyz_aligned: np.ndarray) -> dict:
        """est_xyz_aligned is apply()'s own first return value."""
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
        """est_xyz_aligned/est_q_aligned are apply()'s own two return values."""
        T_est = poses_to_se3(est_xyz_aligned, est_q_aligned)
        T_gt = poses_to_se3(gt_xyz, gt_qxyzw)
        return relative_pose_error_sweep(T_est, T_gt, deltas, xp=xp)
