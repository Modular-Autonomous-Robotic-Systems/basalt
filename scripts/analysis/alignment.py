import numpy as np


def _get_best_yaw(C: np.ndarray) -> float:
    """Closed form theta maximising trace(Rz(theta) . C); C is the 3x3 cross
    covariance data^T . model."""
    a = C[0, 1] - C[1, 0]
    b = C[0, 0] + C[1, 1]
    return np.pi / 2 - np.arctan2(b, a)


def _rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _align_umeyama(model: np.ndarray, data: np.ndarray, known_scale: bool = False,
                    yaw_only: bool = False):
    """Umeyama least squares similarity transform, model = s * R * data + t.
    model and data are Nx3 point sets, rows are points. The SVD of the cross
    covariance is always computed for the scale term, and is separately
    overridden by the closed form yaw only rotation when yaw_only is set."""
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
    """Three Umeyama alignment modes. Every mode takes gt as model and est as
    data, so applying the returned (s, R, t) to est recovers gt."""

    @staticmethod
    def sim3(gt_xyz: np.ndarray, est_xyz: np.ndarray):
        """Monocular VO, scale unobservable, full 7 degree of freedom similarity."""
        return _align_umeyama(gt_xyz, est_xyz, known_scale=False, yaw_only=False)

    @staticmethod
    def se3(gt_xyz: np.ndarray, est_xyz: np.ndarray):
        """Stereo VO, scale metric from the baseline, all three rotation
        degrees of freedom still need aligning."""
        _, R, t = _align_umeyama(gt_xyz, est_xyz, known_scale=True, yaw_only=False)
        return 1.0, R, t

    @staticmethod
    def posyaw(gt_xyz: np.ndarray, est_xyz: np.ndarray):
        """Gravity aligned VIO, roll and pitch already observable via the
        accelerometer and must not be absorbed into alignment, only
        translation and yaw are genuinely unobservable."""
        _, R, t = _align_umeyama(gt_xyz, est_xyz, known_scale=True, yaw_only=True)
        return 1.0, R, t
