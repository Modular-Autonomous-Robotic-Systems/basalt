# Context Directory

This directory contains LLM-generated reference documents for the Basalt SLAM project.
These serve as long-term memory to avoid re-investigating the same questions.

## Documents

| File | Topic | Date |
|------|-------|------|
| `gt_slam_alignment.md` | GT vs SLAM alignment analysis, gravity-alignment explanation, frame geometry, two-part fix — EuRoC + TUM-VI | 2026-07-09/11 |
| `euroc_coordinate_frames.md` | EuRoC dataset coordinate frame reference, GT column decoding | 2026-07-09 |
| `tumvi_coordinate_frames.md` | TUM-VI dataset coordinate frame reference, GT column decoding | 2026-07-11 |
| `vio_localmapper_correction_loop.md` | VIO drift vs local BA correction loop; why live trajectory is never corrected; gauge freedom in NfrMapper BA; path to fix | 2026-07-11 |

## Quick Reference

### GT-SLAM Mismatch Fix (applies to both EuRoC and TUM-VI)
- **Root cause**: World frame origin/orientation mismatch (NOT camera-IMU extrinsics)
- **Fix**: SE(3) first-pose alignment applied in `dataset_io_euroc.h` at IO read time
- **Formula**: `T_gt_aligned[i] = T_gt[0].inverse() * T_gt[i]`

### EuRoC GT Frame
- `state_groundtruth_estimate0` → GT is `T_w_i` (body=IMU, T_BS=Identity confirmed)
- No camera-IMU extrinsics needed for this GT source
- `mocap0` (raw MoCap) → GT is in marker frame; needs `T_imu_marker` (but `T_imu_marker=I` in ds_calib)

### TUM-VI GT Frame
- GT file: `mocap0/data.csv` — **no `state_groundtruth_estimate0`** directory
- GT is `T_w_i` (already converted to IMU frame in EuRoC export)
- **Proof**: `dso/gt_imu.csv` has identical values and explicitly labels the IMU frame
- No sensor.yaml files; calibration via `tumvi_512_ds_calib.json`
- Only room sequences have full-trajectory GT
- 7-column format (no velocity/bias), ~120 Hz Vicon rate

### Downloaded Dataset
- EuRoC: `data/machine_hall/MH_01_easy/`, `data/machine_hall/MH_05_difficult/`
- TUM-VI: `data/TUM/dataset-room1_512_16/` (room1, 512×16 EuRoC export, 1.78 GB)
