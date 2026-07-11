# TUM-VI Dataset Coordinate Frame Reference

**Date**: 2026-07-11
**Source**: Downloaded `dataset-room1_512_16` from https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/
**Paper**: Schubert et al., IROS 2018 — "The TUM VI Benchmark for Evaluating Visual-Inertial Odometry"
**arXiv**: https://arxiv.org/abs/1804.06120

---

## Dataset Variant Covered

This document covers the **TUM-VI dataset exported in EuRoC format** (512×16 resolution variant), specifically the **room sequences** which are the only ones that provide ground truth for the full trajectory.

- **Download URL**: `https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/`
- **Basalt calibration file**: `data/tumvi_512_ds_calib.json`
- **Basalt config file**: `data/tumvi_512_config.json`
- **Basalt dataset type flag**: `--dataset-type euroc` (TUM-VI uses the EuRoC IO!)

Other sequences (corridor, magistrale, slides) only provide partial GT (room entry and exit segments).

---

## Frame Notation (Same as EuRoC — ETH/ASL convention)

`T_AB` = transformation taking a point from frame B to frame A: `p_A = T_AB * p_B`

`p_XY_Z` = position of frame Y origin, expressed in frame Z (with respect to frame X)

---

## TUM-VI Frames

### Body Frame (B) = IMU Frame (I)
- **Definition**: Rigidly attached to the sensor rig, coincident with IMU
- **All sensors** are defined relative to this frame
- The TUM-VI paper states: *"the rounded rectangle contains all components which are rigidly connected with the IMU coordinate system"*

### Reference Frame (R) = World Frame (W)
- **Definition**: Fixed inertial frame of the Motion Capture lab
- **Origin**: Physical Vicon MoCap system origin
- **Gravity**: Approximately aligned with the Z axis

### Camera Frames
- **`T_cam_imu`** in `dso/camchain.yaml` (Kalibr format): transform FROM IMU TO camera
  ```
  cam0: T_cam_imu =
    [-0.9995250379,  0.0296153439, -0.0085223282,  0.0472798822]
    [ 0.0075019185, -0.0343973606, -0.9993800792, -0.0474432321]
    [-0.0298901303, -0.9989693454,  0.0341588513, -0.0681999605]
    [ 0.0,           0.0,           0.0,           1.0         ]
  ```
- **Basalt equivalent**: `T_i_c[0]` = `T_cam_imu.inverse()` (stored in `tumvi_512_ds_calib.json`)
- The large ~180° rotation indicates the cameras are mounted nearly upside-down on the VI sensor rig

---

## Ground Truth Data

### File Locations

| File | Content |
|------|---------|
| `mav0/mocap0/data.csv` | Ground truth poses (EuRoC format) |
| `dso/gt_imu.csv` | **Same data**, explicitly labelled "GT in IMU frame" |

**These two files contain byte-for-byte identical pose data**, confirmed by `diff`. The `dso/gt_imu.csv` naming explicitly confirms the frame.

### Critical Difference from EuRoC

| EuRoC | TUM-VI EuRoC Export |
|-------|-------------------|
| `mav0/state_groundtruth_estimate0/data.csv` | **DOES NOT EXIST** |
| `mav0/mocap0/data.csv` (optional, raw MoCap) | `mav0/mocap0/data.csv` ← **PRIMARY GT source** |
| `mav0/imu0/sensor.yaml` (T_BS = Identity) | **No sensor.yaml files** |

TUM-VI EuRoC export **has no `sensor.yaml` files at all**. Calibration is provided via the `dso/camchain.yaml` (Kalibr format) and the Basalt JSON calibration files.

### CSV Format (`mav0/mocap0/data.csv`)

```
#timestamp [ns], p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []
```

**Column decoding** (same ETH/ASL notation as EuRoC):
- `R` = Reference frame = **World/MoCap frame**
- `S` = Sensor frame = **Body/IMU frame** (after hand-eye calibration)
- `p_RS_R_x/y/z` = position of IMU in world, expressed in world
- `q_RS_w/x/y/z` = quaternion encoding T_world_IMU (w-first format)

**7 columns total** (vs 17 for EuRoC `state_groundtruth_estimate0` which includes velocity and biases)

### First GT Pose (room1 sequence)

```
timestamp: 1520530308189679351 ns
position:  (0.8418, -0.2193, 1.2500) m   ← NOT at origin!
quaternion (w,x,y,z): (0.9996, 0.0037, 0.0097, -0.0243)
```
The near-identity quaternion means the sensor is approximately upright/level at sequence start.
The non-zero position (~(0.84, -0.22, 1.25)m) is the location in the MoCap room.

### Last GT Pose (room1 sequence)

```
timestamp: 1520530449214678351 ns
position:  (0.1663, -0.0718, 1.0918) m
quaternion (w,x,y,z): (0.0213, -0.0472, 0.0381, 0.9979)
```

### GT Rate

16,541 poses over ~141 seconds → approximately 117 Hz (close to the Vicon 120 Hz rate).

---

## How Basalt Reads TUM-VI Ground Truth

In `include/basalt/io/dataset_io_euroc.h`, `EurocIO::read()`:

```cpp
// Priority fallthrough for GT source:
if (!load_mocap_as_gt &&
    fs::exists(path + "/mav0/state_groundtruth_estimate0/data.csv")) {
    read_gt_data_state(path + "/mav0/state_groundtruth_estimate0/");  // EuRoC path
} else if (!load_mocap_as_gt &&
           fs::exists(path + "/mav0/gt/data.csv")) {
    read_gt_data_pose(path + "/mav0/gt/");                             // alternate
} else if (fs::exists(path + "/mav0/mocap0/data.csv")) {
    read_gt_data_pose(path + "/mav0/mocap0/");                         // TUM-VI path
}
```

TUM-VI falls into the **third branch** — it uses `read_gt_data_pose()` reading `mocap0/data.csv`.

`read_gt_data_pose()` reads **7 columns**: `timestamp, px, py, pz, qw, qx, qy, qz`
`read_gt_data_state()` reads **17 columns**: additionally velocity + bias terms

No `T_imu_marker` correction is applied in the code — consistent with `T_imu_marker = Identity` in `tumvi_512_ds_calib.json`.

---

## GT Frame: Raw MoCap vs. Calibrated IMU Frame

**The TUM-VI paper** (Schubert et al.) describes the pipeline:

1. Raw MoCap system tracks reflective markers → `T_WM` (world to marker frame)
2. Hand-eye calibration estimates `T_MI` (marker to IMU frame)
3. Calibrated GT = `T_WI = T_WM * T_MI`

**The EuRoC export ALREADY applies step 3.** Evidence:
- `dso/gt_imu.csv` ≡ `mav0/mocap0/data.csv` (byte-identical)
- `gt_imu.csv` is explicitly named "GT in IMU frame"
- Basalt calibration: `T_imu_marker = Identity` (marker frame ≡ IMU frame in the export)

Therefore: `mocap0/data.csv` = `T_world_IMU` = `T_w_i` in Basalt notation.

**This is identical to EuRoC `state_groundtruth_estimate0` in terms of frame semantics.**

---

## Basalt Calibration Conventions for TUM-VI

From `tumvi_512_ds_calib.json`:
- `T_imu_cam[0]`: Transform FROM camera 0 TO IMU frame (Basalt's `T_i_c[0]`)
- `T_mocap_world = Identity`: MoCap world ≡ SLAM world
- `T_imu_marker = Identity`: Marker frame ≡ IMU frame (in exported format)

Compared to EuRoC `euroc_ds_calib.json`:
- Same structure, different values
- Both have `T_mocap_world = Identity` and `T_imu_marker = Identity`

---

## Key Differences: TUM-VI vs EuRoC

| Property | EuRoC | TUM-VI (EuRoC export) |
|----------|-------|----------------------|
| GT CSV source | `state_groundtruth_estimate0/` | `mocap0/` |
| GT CSV columns | 17 (pos + quat + vel + bias) | **7** (pos + quat only) |
| sensor.yaml files | Present in every subdir | **Absent** |
| GT throughout | Full sequence | **Room sequences only** (120 Hz at start/end for others) |
| Calibration files | `euroc_ds_calib.json` | `tumvi_512_ds_calib.json` |
| GT rate | 200 Hz (Leica+IMU batch) | ~120 Hz (Vicon MoCap) |
| GT Frame | T_world_body (body=IMU) | T_world_body (body=IMU) ← **same** |
| Alignment needed | First-pose SE3 | First-pose SE3 ← **same fix** |

---

## Alignment Requirement for Visualisation

**Root cause** (identical to EuRoC): SLAM initialises at `T_w_i = Identity` but GT starts at a non-origin pose in the MoCap lab frame.

**Fix** (identical to EuRoC — first-pose alignment):
```cpp
Sophus::SE3d T_aligned = T_gt_first_pose.inverse() * T_gt_current;
```

The same visualiser fix in `src/visualisation/visualiser.cpp:Run()` handles both EuRoC and TUM-VI without any dataset-specific branching.

---

## Dataset Structure Summary

```
dataset-room1_512_16/
├── mav0/                        # EuRoC-compatible structure
│   ├── cam0/
│   │   ├── data/                # 512×512 fisheye images (PNG)
│   │   └── data.csv             # Image timestamps
│   ├── cam1/
│   │   ├── data/                # 512×512 fisheye images (PNG)
│   │   └── data.csv
│   ├── imu0/
│   │   └── data.csv             # IMU measurements (6-axis, 200 Hz)
│   └── mocap0/
│       └── data.csv             # GT poses T_world_IMU (7-col, ~120 Hz)
└── dso/                         # DSO-compatible structure
    ├── cam0/                    # camera.txt (EquiDist model), pcalib.txt
    ├── cam1/
    ├── camchain.yaml            # Kalibr: T_cam_imu for both cameras
    ├── imu_config.yaml          # IMU noise parameters
    ├── gt_imu.csv               # Same GT as mocap0/data.csv (explicit IMU label)
    └── imu.txt
```

**Note**: No `state_groundtruth_estimate0/` directory exists in TUM-VI EuRoC export.

---

## Sources

- TUM-VI paper: [Schubert et al. "The TUM VI Benchmark" (IROS 2018)](https://arxiv.org/abs/1804.06120)
- Dataset download: [TUM CVG Visual-Inertial Dataset](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset)
- Basalt docs: [basalt/doc/VioMapping.md (GitHub)](https://github.com/VladyslavUsenko/basalt/blob/master/doc/VioMapping.md)
- Local files:
  - `data/TUM/dataset-room1_512_16/mav0/mocap0/data.csv`
  - `data/TUM/dataset-room1_512_16/dso/gt_imu.csv`
  - `data/TUM/dataset-room1_512_16/dso/camchain.yaml`
  - `data/tumvi_512_ds_calib.json`
