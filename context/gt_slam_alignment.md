# Ground Truth vs SLAM Trajectory Alignment Analysis

**Date**: 2026-07-09 (EuRoC analysis) — Updated 2026-07-11 (TUM-VI analysis, gravity-alignment explanation, frame geometry)
**Issue**: GT (green) and SLAM (red/pink) trajectories are not aligned in the local mapping visualisation window.
**Reference image**: `/ws/gt-slam-mismatch.png`

---

## 1. EuRoC Ground Truth Frame of Reference

### What frame is the EuRoC GT in?

**Answer**: The EuRoC `state_groundtruth_estimate0` ground truth is `T_world_body` where **body = IMU frame**.

This is confirmed by two pieces of evidence from the actual dataset files:

**`mav0/state_groundtruth_estimate0/sensor.yaml`**:
```yaml
# Sensor extrinsics wrt. the body-frame.
T_BS:
  data: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]  # Identity!
```

**`mav0/imu0/sensor.yaml`**:
```yaml
# Sensor extrinsics wrt. the body-frame.
T_BS:
  data: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]  # Identity!
```

Both the GT sensor and the IMU have `T_BS = Identity`, meaning both are identical to the body frame.

### Column header decoding (ETH/ASL notation)

The CSV header for `state_groundtruth_estimate0/data.csv`:
```
#timestamp, p_RS_R_x, p_RS_R_y, p_RS_R_z, q_RS_w, q_RS_x, q_RS_y, q_RS_z, ...
```

In ETH/ASL frame notation:
- `R` = Reference frame = **World frame** (fixed Vicon/Leica lab frame)
- `S` = Sensor frame = **Body/IMU frame**
- `p_RS_R` = position of frame S origin, expressed in frame R = **position of IMU in world**
- `q_RS` = quaternion encoding rotation from S to R = **rotation part of T_world_IMU**

Therefore: `GT_data = T_world_IMU = T_w_i` in Basalt notation.

**First GT pose in MH_01_easy**:
```
timestamp: 1403636580838555648
position:  (4.688, -1.787, 0.783) meters
quaternion (w,x,y,z): (0.534, -0.153, -0.827, -0.082)
```

### Sources
- Official EuRoC paper: Burri et al., IJRR 2016 — https://journals.sagepub.com/doi/abs/10.1177/0278364915620033
- Local sensor.yaml files in `/ws/ros_ws/src/slam/ext/basalt/data/machine_hall/`
- ORB-SLAM3 EuRoC documentation: https://mintlify.wiki/UZ-SLAMLab/ORB_SLAM3/examples/euroc-dataset

---

## 1b. TUM-VI Ground Truth Frame of Reference

### What dataset variant?

This analysis covers the **TUM-VI dataset in EuRoC export format** (512×16 resolution), specifically the **room sequences** which are the only sequences providing full-trajectory GT. Basalt reads TUM-VI using the **same `EurocIO` class** as EuRoC (dataset type flag: `--dataset-type euroc`).

Downloaded from: `https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/`
Analyzed sequence: `dataset-room1_512_16`

### What frame is the TUM-VI GT in?

**Answer**: `T_world_body` where **body = IMU frame** — identical semantics to EuRoC.

**Confirmed by two independent sources from the actual dataset files:**

**1. `mav0/mocap0/data.csv` column header:**
```
#timestamp [ns], p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []
```
Same ETH/ASL notation as EuRoC where R=world, S=body/IMU → poses are `T_world_IMU`.

**2. `dso/gt_imu.csv` — explicit IMU label:**
```
# timestamp[ns],tx,ty,tz,qw,qx,qy,qz
1520530308189679351,0.8417820384,...  ← identical values to mocap0/data.csv
```
The file is named `gt_imu.csv` and is explicitly labelled as "GT in IMU frame". The values are **byte-for-byte identical** to `mocap0/data.csv` (verified by `diff`).

**3. Basalt calibration `tumvi_512_ds_calib.json`:**
```json
"T_imu_marker": {"px":0.0, "py":0.0, "pz":0.0, "qx":0.0, "qy":0.0, "qz":0.0, "qw":1.0}
```
`T_imu_marker = Identity` → marker frame ≡ IMU frame in the exported format. No marker-to-IMU conversion needed.

### TUM-VI GT pipeline (from Schubert et al. IROS 2018)

The raw data pipeline is:
1. Vicon tracks reflective markers → `T_WM` (world to marker frame)
2. Hand-eye calibration estimates `T_MI` (marker to IMU transform)
3. Calibrated GT = `T_WI = T_WM * T_MI`

The **EuRoC export already applies step 3**. What `mocap0/data.csv` stores is `T_WI = T_world_IMU`.

### No sensor.yaml in TUM-VI EuRoC export

**Critical difference from EuRoC**: TUM-VI EuRoC format has **no sensor.yaml files** in any subdirectory. The archive only contains:
- `mav0/cam0/data.csv`, `mav0/cam1/data.csv` — image timestamps
- `mav0/imu0/data.csv` — IMU measurements
- `mav0/mocap0/data.csv` — **GT poses** (7-column: ts + pos + quat)
- `dso/` directory — DSO format (camera.txt, camchain.yaml, gt_imu.csv, imu.txt)

Calibration is provided externally via `data/tumvi_512_ds_calib.json`.

### CSV format differences: TUM-VI vs EuRoC

| Property | EuRoC `state_groundtruth_estimate0` | TUM-VI `mocap0` |
|----------|--------------------------------------|-----------------|
| Columns | **17** (ts + pos3 + quat4 + vel3 + bias6) | **7** (ts + pos3 + quat4) |
| Rate | ~200 Hz (batch Leica+IMU solution) | ~120 Hz (Vicon MoCap) |
| Basalt reader | `read_gt_data_state()` | `read_gt_data_pose()` |
| Full sequence GT | Always | **Room sequences only** |

### Column header decoding (same ETH/ASL notation)

```
p_RS_R   → position of Sensor S in Reference R (world), expressed in R
q_RS     → quaternion: rotation part of T_world_body
```
Therefore: `GT data = T_world_IMU = T_w_i` in Basalt notation ✓

### First GT pose in room1 sequence

```
timestamp: 1520530308189679351 ns
position:  (0.8418, -0.2193, 1.2500) m  ← in MoCap room, NOT at origin
quaternion (w,x,y,z): (0.9996, 0.0037, 0.0097, -0.0243)
```
Near-identity quaternion → sensor is approximately level/upright. But position is ~0.84m, -0.22m, 1.25m from MoCap origin. Same world-frame origin mismatch problem as EuRoC.

### How EurocIO routes to the TUM-VI GT

```cpp
// dataset_io_euroc.h, EurocIO::read():
if (!load_mocap_as_gt &&
    fs::exists(path + "/mav0/state_groundtruth_estimate0/data.csv")) {
    read_gt_data_state(/* 17-col */);           // ← EuRoC path
} else if (!load_mocap_as_gt &&
           fs::exists(path + "/mav0/gt/data.csv")) {
    read_gt_data_pose(/* 7-col */);
} else if (fs::exists(path + "/mav0/mocap0/data.csv")) {
    read_gt_data_pose(/* 7-col */);             // ← TUM-VI path (falls through here)
}
```

### Sources
- TUM-VI paper: Schubert et al., IROS 2018 — https://arxiv.org/abs/1804.06120
- Local files: `data/TUM/dataset-room1_512_16/mav0/mocap0/data.csv`
- Local files: `data/TUM/dataset-room1_512_16/dso/gt_imu.csv` (explicit IMU label)
- Local calibration: `data/tumvi_512_ds_calib.json`

---

## 2. Root Cause of the Trajectory Mismatch

### The actual problem

The user suspected camera-IMU extrinsics as the cause. **This is incorrect for EuRoC `state_groundtruth_estimate0`.**

The EuRoC GT is already in the IMU body frame (`T_w_i`), matching Basalt's output convention exactly. The mismatch is a **world frame origin and orientation difference**:

| | SLAM World Frame | GT World Frame |
|---|---|---|
| Origin | Where IMU was at t=0 | Fixed Vicon/Leica lab origin |
| Orientation | Aligned with initial IMU orientation | Fixed lab orientation |
| First pose | Identity (by design) | (4.688, -1.787, 0.783) + complex rotation |

The SLAM system in `basalt_slam.cpp` explicitly initialises at identity:
```cpp
controller->initialize(0,              // t_ns
                       Sophus::SE3d(), // T_w_i = Identity
                       ...);
```

### Visualisation in the code

In `src/visualisation/visualiser.cpp`:
```cpp
// GT trajectory (line 171) — raw poses, no alignment:
while (mvpGroundTruthQueue.try_pop(gt))
    mvpGroundTruthTrajectory.emplace_back(gt.T_w_i.translation());

// SLAM trajectory (line 449) — also raw poses:
mvpVioTrajectory.emplace_back(data->T_w_i.translation());
```

Both trajectories are stored in their own world frames, which do NOT share an origin or orientation. This produces the dramatic misalignment visible in the screenshot.

---

## 3. Required Fix: SE(3) First-Pose Alignment

### Mathematical derivation

> ⚠️ **Important caveat**: The derivation below assumes `T_{W_slam, IMU}(t_0) = I`. This is
> only true if the SLAM system does NOT gravity-align its world frame at startup. Basalt
> **does** gravity-align, so `T_{W_slam, IMU}(t_0) = T_slam_first ≠ I`. The IO-level fix
> below (Part 1) is necessary but not sufficient; see **Section 5b** for the full two-part fix.

The SLAM world frame `W_slam` is defined by the condition `T_{W_slam, IMU}(t_0) = I`.
The GT world frame `W_gt` is fixed in the lab: `T_{W_gt, IMU}(t_0) = T_gt[0]`.

To express GT poses in the SLAM world frame, compute the alignment transform:
```
T_{W_slam, W_gt} = T_{W_slam, IMU}(t_0) * T_{IMU, W_gt}(t_0)
                 = I * T_gt[0].inverse()
                 = T_gt[0].inverse()
```

Then for each GT pose at time t:
```
T_{W_slam, IMU}(t) = T_{W_slam, W_gt} * T_{W_gt, IMU}(t)
                   = T_gt[0].inverse() * T_gt[i]
```

In code (applied at IO read time in `dataset_io_euroc.h`):
```cpp
// In read_gt_data_state() and read_gt_data_pose():
Sophus::SE3d T0_inv;
bool first = true;
while (std::getline(f, line)) {
    // ... parse timestamp, q, pos ...
    Sophus::SE3d pose(q, pos);
    if (first) { T0_inv = pose.inverse(); first = false; }

    data->gt_timestamps.emplace_back(timestamp);
    data->gt_pose_data.emplace_back(T0_inv * pose);
}
```

This approach normalises the trajectory at read time so that the identity-origin guarantee
holds everywhere in the codebase (evaluation, logging, visualisation), not just in the
rendering path.

### Why NOT camera-IMU extrinsics?

Camera-IMU extrinsics (`T_i_c` in Basalt) would be needed if:
- The GT was provided in the **camera frame** (e.g., KITTI)
- The GT was provided in a **marker frame** ≠ IMU frame (e.g., EuRoC `mocap0`)

For EuRoC `state_groundtruth_estimate0`, the GT frame IS the IMU frame, so extrinsics are irrelevant.

**Where camera-IMU extrinsics ARE needed for GT conversion**:
```
GT in camera frame → T_w_i = GT * T_i_c[0].inverse()
GT in mocap marker frame → T_w_i = GT * T_imu_marker (from calib JSON)
```

---

## 4. Cross-Dataset GT Frame Conventions

| Dataset | GT source file | GT frame | Extra sensor transform | Notes |
|---------|---------------|----------|----------------------|-------|
| EuRoC `state_groundtruth_estimate0` | `state_groundtruth_estimate0/data.csv` | T_world_body (body=IMU) | **None** | T_BS=Identity confirmed; 17-col CSV |
| EuRoC `mocap0` (if using `load_mocap_as_gt`) | `mocap0/data.csv` | T_world_marker | `T_imu_marker` from calib JSON | Marker ≠ IMU frame |
| **TUM-VI (EuRoC export)** | **`mocap0/data.csv`** | **T_world_body (body=IMU)** | **None** | Exported with hand-eye calib applied; 7-col CSV; `T_imu_marker=I` |
| KITTI | `poses/XX.txt` | T_world_cam0 | `T_i_c[0]` (cam→IMU) | GT in left camera frame |
| Custom/ROS | Varies | Topic-dependent | Configure per-dataset | Check frame_id |

### TUM-VI specific facts (confirmed from dataset files)

- GT file path: `mav0/mocap0/data.csv` — no `state_groundtruth_estimate0`
- **Identical** to `dso/gt_imu.csv` (same data, explicit "IMU" label)
- Format: 7 columns only (`timestamp, px, py, pz, qw, qx, qy, qz`)
- No sensor.yaml files anywhere in the archive
- Basalt reads it via `read_gt_data_pose()` fallthrough in `EurocIO::read()`
- First pose room1: `(0.8418, -0.2193, 1.2500)` m, q≈identity (sensor upright)

### EuRoC vs TUM-VI quick comparison

| | EuRoC | TUM-VI |
|--|-------|--------|
| GT directory | `state_groundtruth_estimate0/` | `mocap0/` |
| sensor.yaml | Present (T_BS=I confirmed) | **Absent** |
| GT columns | 17 (includes vel + bias) | 7 (pos + quat only) |
| GT rate | 200 Hz | ~120 Hz |
| GT throughout | Full sequence | Room sequences only |
| First-pose alignment needed | ✓ | ✓ |
| Camera-IMU extrinsics needed | ✗ | ✗ |

### Generalisation strategy

For a dataset-agnostic solution, add a `gt_frame` configuration per dataset type:
```cpp
enum class GtFrame { IMU_BODY, CAMERA_0, MOCAP_MARKER, CUSTOM };
```

Then in the dataset reader or visualiser, apply the appropriate sensor-to-IMU transform before storing/displaying GT poses. Finally, always apply **first-pose alignment** regardless of dataset.

### Should we visualise T_w_c instead of T_w_i?

**No.** Reasons:
1. VIO systems natively estimate T_w_i; T_w_c is a derived quantity
2. EuRoC and TUM-VI (the primary benchmarks) both provide GT in IMU body frame — confirmed from actual dataset files
3. SLAM evaluation tools (evo, etc.) typically expect body/IMU frame poses
4. The visualiser already renders camera frusta by multiplying: `T_w_i * T_i_c[i]`

For pure VO systems (no IMU), T_w_c is the natural choice, but for VIO, T_w_i is correct.

---

## 5. Where the Fix Was Applied

### Primary fix location: `include/basalt/io/dataset_io_euroc.h`

Normalisation is applied inline during CSV parsing in both GT reading methods:

- `read_gt_data_state()` (lines ~266–297) — used for `state_groundtruth_estimate0`
- `read_gt_data_pose()` (lines ~299–327) — used for `mocap0` / pose-only GT

Normalising at IO time means `gt_pose_data` always contains first-pose-relative poses
regardless of which consumer reads them (visualiser, evaluation scripts, logging, etc.).
No changes are needed in the visualiser or any other consumer.

### Other dataset readers

**KITTI** (`dataset_io_kitti.h`): GT is in camera frame — convert first, then normalise:
```cpp
// gt_pose_data[i] = calib.T_i_c[0] * raw_cam_pose;  // cam0→IMU
```

**Custom dataset**: Document expected GT frame convention and apply the same pattern.

---

## 5b. Residual 90° Rotation After IO-Level Fix (and its fix)

**Symptom**: After applying the IO-level first-pose alignment, the trajectories share the same starting point but the GT shape is ~90° rotated relative to the SLAM shape.

**Root cause**: Basalt's `initialize(bg, ba)` branch (called when `t_ns==0` in `controller.cpp`) sets the initial pose by gravity-aligning the world frame:

```cpp
// sqrt_keypoint_vio.cpp:248
T_w_i_init.setQuaternion(Eigen::Quaternion<Scalar>::FromTwoVectors(
    imuData->accel, Vec3::UnitZ()));
```

`FromTwoVectors(accel, +Z)` rotates so the anti-gravity direction in the IMU body frame maps to world +Z. This is NOT Identity — it is the gravity-alignment rotation. For EuRoC MH_01_easy, the MAV's IMU X axis ≈ world Z (upward) when hovering: anti-gravity ≈ `+X_imu`, so:

```
FromTwoVectors(+X_imu, +Z_world) = 90° rotation about −Y
```

The IO fix defines the GT world frame as the initial IMU body frame (Identity). The SLAM world frame is the gravity-aligned frame (~90° from Identity). The visualiser rendered both uncorrected → 90° mismatch.

**Fix** (`src/visualisation/visualiser.cpp`): Capture the first SLAM pose `T_slam_first` in `ConsumeVioStateQueue()`. In `Run()`, snapshot it under `mpMtxVioState` and multiply every IO-normalised GT pose by it before plotting:

```cpp
// In ConsumeVioStateQueue (under mpMtxVioState lock):
if (!mpSlamFirstPose) mpSlamFirstPose = data->T_w_i;

// In Run() GT drain (snapshot then apply):
std::optional<Sophus::SE3d> slam_first;
{ std::lock_guard lock(mpMtxVioState); slam_first = mpSlamFirstPose; }
while (mvpGroundTruthQueue.try_pop(gt)) {
    Sophus::SE3d T_gt_vis = slam_first ? *slam_first * gt.T_w_i : gt.T_w_i;
    mvpGroundTruthTrajectory.emplace_back(T_gt_vis.translation());
}
```

Combined with the IO fix this gives the full alignment:
```
T_gt_vis[i] = T_slam_first * T_gt[0].inverse() * T_gt_raw[i]
```

This works for any dataset regardless of how much the gravity vector differs from the IMU's Z axis.

---

## 5c. Why Basalt Gravity-Aligns the SLAM World Frame

### What is the "world frame"?

`T_w_i` = pose of the IMU body expressed **in the world frame**. The world frame is not
physical — it is a **design choice** made by the VIO algorithm at startup. Two systems can
both output `T_w_i` and still be incompatible if they chose different world frame definitions.

### Two options for the SLAM world frame

| Option | World frame definition | First T_w_i | Gravity in world |
|--------|----------------------|-------------|-----------------|
| **A: Arbitrary** | = initial IMU body orientation | **Identity** | `(gx, gy, gz)` — varies, must track |
| **B: Gravity-aligned** (Basalt) | Z_world = anti-gravity (UP) | **≠ Identity** | `(0, 0, −9.81)` — constant |

Basalt uses **Option B**. The rationale is IMU pre-integration: at every step the
accelerometer reading must have gravity subtracted:

```
a_true_body = a_measured_body − g_body
           = a_measured_body − R_w_i^T * g_world
```

If the world frame is gravity-aligned, `g_world = (0, 0, −9.81)` is a constant. If the
world frame is arbitrary (Option A), `g_world` is a vector that must be kept in the state
and propagated. Option B is numerically cleaner, produces maps with Z = up (human-readable),
and matches the assumptions in many VIO derivations.

**The tradeoff**: `T_w_i_first ≠ Identity`. The first output pose encodes the actual physical
tilt of the IMU relative to the gravity-aligned world frame. For EuRoC, this tilt is ~90°.

### The gravity-alignment code

In `src/vi_estimator/sqrt_keypoint_vio.cpp:248`, when `initialize(bg, ba)` is called
(the branch taken in `controller.cpp:155` when `t_ns == 0`):

```cpp
T_w_i_init.setQuaternion(Eigen::Quaternion<Scalar>::FromTwoVectors(
    imuData->accel, Vec3::UnitZ()));
```

`FromTwoVectors(a, b)` = the minimal rotation taking vector `a` to vector `b`. When the
MAV is approximately stationary, `imuData->accel` ≈ anti-gravity in the IMU body frame
(specific force = measured acceleration − gravity). The rotation takes the anti-gravity
direction to `+Z_world`, ensuring `g_world = (0, 0, −9.81)` thereafter.

### Yaw is preserved, only pitch/roll are corrected

`FromTwoVectors` computes the **minimum** rotation between two vectors. Its rotation axis
lies in the plane spanned by the two vectors, which is perpendicular to the gravity axis.
This means:

- The yaw (rotation about the gravity/world-Z axis) is **not changed** from the initial
  IMU body orientation
- Only pitch and roll (tilt relative to gravity) are corrected
- If two runs start with the MAV pointing in different compass headings, their SLAM worlds
  will have different XY orientations but the same Z = up

This yaw-unobservability is fundamental to monocular VIO: without a compass or prior map,
the absolute heading cannot be recovered.

---

## 5d. Frame Geometry at Startup (EuRoC MH_01_easy)

This section gives the concrete frame relationships for EuRoC MH_01_easy as a worked example.
The general principle is: body/IMU frame is fixed to the sensor, camera frame to the lens,
and the SLAM world frame is set by gravity-alignment.

### IMU body frame in the EuRoC lab (Vicon world)

From the first GT quaternion `(w=0.534, x=−0.153, y=−0.827, z=−0.082)`, R_w_i has columns:

```
X_imu in Vicon world ≈ (−0.38,  0.17,  0.91)  ≈  +Z_lab  (UPWARD)
Y_imu in Vicon world ≈ ( 0.34,  0.94, −0.03)  ≈  +Y_lab  (horizontal)
Z_imu in Vicon world ≈ (−0.86,  0.30, −0.41)  ≈  −X_lab  (forward in corridor)
```

So at the start of MH_01_easy the MAV has its **IMU X axis pointing straight up**. This is
an atypical mounting (standard FRD drones have Z pointing down; EuRoC's IMU has X pointing
up). Gravity in the IMU body frame ≈ `−X_imu`, so anti-gravity ≈ `+X_imu`.

### Gravity-alignment rotation

`FromTwoVectors(+X_imu, +Z_world)` = 90° rotation about the −Y axis:

```
R_w_i_init = [[ 0,  0, -1],
              [ 0,  1,  0],
              [ 1,  0,  0]]
```

Verify: `R * (1,0,0) = (0,0,1)` ✓ (X_imu → Z_slam = up)

### SLAM world axes in IMU body terms

| SLAM world axis | IMU body direction | Physical meaning |
|---|---|---|
| +X_slam | −Z_imu | backward on the MAV |
| +Y_slam | +Y_imu | rightward |
| +Z_slam | +X_imu | UP (anti-gravity) ✓ |

Forward motion in the corridor (= +Z_imu direction) maps to **−X_slam** in the SLAM world.

### Camera frame in the SLAM world

From `T_BS` for cam0 (= `T_imu_cam` in `euroc_ds_calib.json`, qz≈0.702 qw≈0.712 ≈ 90° about Z_imu):

```
X_cam ≈ +Y_imu  →  +Y_slam    (image right  = world right)
Y_cam ≈ −X_imu  →  −Z_slam    (image down   = world down ✓)
Z_cam ≈ +Z_imu  →  −X_slam    (optical axis = world "forward")
```

Diagram in the initial SLAM world:

```
         ↑ Z_slam (UP, anti-gravity)
         │
         │    Camera at origin, looking into −X_slam (forward)
         │
         └──────────────────→ Y_slam (right)
        /
      −X_slam  ← camera optical axis (MAV flying this way)
```

The camera image is right-side up (image-down = world-down = −Z_slam) and the MAV
trajectory in MH_01_easy appears as a loop in the XY (horizontal) plane of the SLAM world.

### The `T_imu_cam ≈ 90° about Z` connection

The `euroc_ds_calib.json` `T_imu_cam[0]` quaternion `(qz≈0.702, qw≈0.712)` is a ~90°
rotation about Z_imu. This yaw offset between the camera and IMU frames does NOT directly
cause the trajectory mismatch — but both stem from the same MAV geometry: the IMU is mounted
with X up and Z forward, so the camera (Z_cam ≈ Z_imu = forward) is rotated 90° about the
vertical axis relative to the IMU's XY axes. The trajectory 90° mismatch is caused by the
gravity-alignment acting on IMU X (up) → SLAM Z, not by the camera-IMU extrinsic.

### Summary: which axis maps to which at SLAM startup

| Physical direction | IMU body | Camera | SLAM world |
|---|---|---|---|
| UP (anti-gravity) | X | −Y | **Z** |
| Forward (corridor) | Z | Z | −X |
| Right | −Y | X | Y |

---

## 5e. Standard NED/FRD mounting (SITL/ArduPilot) hits the degenerate branch of `FromTwoVectors`, so the world's horizontal axes carry no fixed meaning (2026-09-19)

The worked example in §5d is specific to EuRoC's unusual mounting, where the IMU's X axis points up. That makes the stationary `accel` vector sit at roughly 90° to `Vec3::UnitZ()` in `T_w_i_init.setQuaternion(FromTwoVectors(imuData->accel, Vec3::UnitZ()))` (`sqrt_keypoint_vio.cpp:267-268`), a well-conditioned input. The `djinn start sitl2` stack has the opposite, and worse-conditioned, geometry: the body frame is standard NED/FRD (X-forward, Y-right, Z-**down**, confirmed in `ap_dds_imu_stream.md`), so a level, stationary vehicle reads `accel ≈ (0, 0, −g)`, almost exactly **antiparallel** to the `(0, 0, 1)` target. `FromTwoVectors`'s rotation axis is `normalize(accel × UnitZ) ≈ normalize(accel_y, −accel_x, 0)`, so near this antiparallel limit the axis direction, and therefore which combination of body-X/body-Y ends up rotated into world-X/world-Y, is set by whichever residual horizontal component (accelerometer noise, order of a few mg) the very first sample happens to carry. Z is unaffected by this, the rotation magnitude (~180°, flipping down to up) is well-conditioned regardless.

Confirmed empirically from four independent `Setting up filter` events captured in `/ws/log.log` and `/ws/log2.log` (each prints the full `T_w_i` matrix). Reading each matrix's first column as "body-forward (X) expressed in world coordinates" and the third as "body-down (Z) expressed in world coordinates":

| Log / `t_ns` | `X_body` in world | `Z_body` in world | Yaw of `X_body` (atan2) |
|---|---|---|---|
| `log2.log` / 44142000000 | (0.8748, 0.4844, −0.0017) | (−0.0017, 0.0066, −1.0000) | 28.98° |
| `log.log` / 48822000000 | (0.2595, 0.9657, 0.0043) | (−0.0043, 0.0057, −1.0000) | 74.98° |
| `log2.log` / 151056000000 | (0.4642, −0.8857, −0.0040) | (0.0040, 0.0066, −1.0000) | −62.36° |
| `log2.log` / 463740000000 | (−0.1355, 0.9908, −0.0050) | (0.0050, −0.0043, −1.0000) | 97.80° |

Every `Z_body` column is (≈0, ≈0, −0.9999+) to sub-degree precision across all four, so "world Z = up" is a solid, repeatable invariant. `X_body`'s direction in the world's horizontal plane spans roughly 160° across the four samples with no discernible pattern, so "world X = forward" or "world Y = right" is **not** a property this code establishes, it is an artefact of the first sample's noise.

**Exact closed form (2026-09-19, sharpens the above).** Decomposing each matrix as intrinsic ZYX Euler angles (roll about X, then pitch about Y, then yaw about Z, `R = Rz(yaw)·Ry(pitch)·Rx(roll)`, the same convention used throughout `airsim_camera_extrinsics.md`) gives roll ≈ 180°, pitch ≈ 0° on every one of the four samples, with only yaw varying (74.98°, 28.98°, −62.36°, 97.80° respectively, matching a direct Euler decomposition to within a fraction of a degree). This is not four coincidences, it is an algebraic identity. A rotation by exactly 180° about any horizontal axis `n = (cos α, sin α, 0)` has the closed form `R = Rz(2α)·Rx(180°)` (proof: Rodrigues' formula gives `R = 2nnᵀ − I` for a π rotation, which expands to exactly this product). Since `FromTwoVectors(accel, UnitZ())` in the level-at-start case computes a rotation whose angle is forced to ≈180° (the antiparallel condition) about a horizontal axis `n` whose direction `α` is set by the first sample's residual noise, **`T_w_i_init`'s rotation is structurally `Rz(2α)·Rx(180°)` every time**: roll = 180° and pitch = 0° are mathematical necessities of this degenerate branch, not measured or run-dependent, and yaw = 2α is the one genuinely free parameter, fixed by the azimuth of a sub-mg noise vector.

**This yaw does not align with, approximate, or offset toward compass north, or toward the vehicle's own heading, in any way.** `FromTwoVectors` takes only the accelerometer reading and the fixed target `(0,0,1)` as input, there is no magnetometer, no compass, no heading reference anywhere in this computation, and no code path from a spawn heading or a GPS course into it. Reading a `74.98°` yaw as "the offset needed to match north" mistakes an accelerometer-noise artefact for a deliberate calibration; the correct reading is that this yaw is unpredictable and run-to-run meaningless, exactly the point the empirical table above establishes. Consequence: the visualiser's world-origin triad (see `visualiser_pipeline.md`) reliably shows blue (Z) pointing up, but red (X) and green (Y) point in an essentially arbitrary, run-dependent horizontal direction, not a fixed "forward"/"right" relative to the vehicle. This does not affect GT alignment correctness, since the existing per-run first-pose alignment (§3, §5b) always aligns against that same run's own arbitrary `T_w_i_init`, but it does mean no code or human should read the visualiser's horizontal axes, or any fixed offset from them, as a stable compass-like convention across runs. A fix, if wanted, would seed yaw from something better-conditioned than a near-vertical accelerometer sample (e.g. a compass/magnetometer, or the known spawn heading), left as an open item, not implemented here.

---

## 6. Verification

After applying first-pose alignment, the trajectories should overlap at t=0 and diverge only due to SLAM drift. To validate:

1. **Visual**: GT (green) and SLAM (red) should start at the same point in the viewer
2. **Metric**: Use `evo_ape` with `--align_origin` flag or full SE3 alignment
3. **Code check**: `T_gt[0].inverse() * T_gt[0]` should be Identity (zero translation, identity rotation)

---

## 7. Related Files

### Core implementation

| File | Role |
|------|------|
| `include/basalt/io/dataset_io_euroc.h:266` | `read_gt_data_state` — normalises GT inline during CSV parse (EuRoC) |
| `include/basalt/io/dataset_io_euroc.h:299` | `read_gt_data_pose` — same normalisation for mocap/pose-only GT (TUM-VI) |
| `include/basalt/visualisation/utils.h:31` | `GtPose` struct definition |
| `src/basalt_slam.cpp:82` | GT pose lookup and forwarding |
| `src/controller.cpp:196` | GT forwarded to vis queue |
| `src/visualisation/visualiser.cpp:169` | GT trajectory buffer |

### EuRoC evidence

| File | Evidence |
|------|---------|
| `data/machine_hall/*/mav0/state_groundtruth_estimate0/sensor.yaml` | T_BS=Identity → GT frame = body = IMU |
| `data/machine_hall/*/mav0/imu0/sensor.yaml` | T_BS=Identity → IMU frame = body |
| `data/euroc_ds_calib.json` | T_imu_cam, T_mocap_world=I, T_imu_marker=I |

### TUM-VI evidence

| File | Evidence |
|------|---------|
| `data/TUM/dataset-room1_512_16/mav0/mocap0/data.csv` | Primary GT, same column headers as EuRoC |
| `data/TUM/dataset-room1_512_16/dso/gt_imu.csv` | Same values, **explicitly names IMU frame** |
| `data/TUM/dataset-room1_512_16/dso/camchain.yaml` | T_cam_imu calibration in Kalibr format |
| `data/tumvi_512_ds_calib.json` | T_imu_cam[0/1], T_mocap_world=I, T_imu_marker=I |

### Context documents

| File | Topic |
|------|-------|
| `context/gt_slam_alignment.md` | This file — cross-dataset analysis |
| `context/euroc_coordinate_frames.md` | EuRoC frame notation reference |
| `context/tumvi_coordinate_frames.md` | TUM-VI frame notation reference |

---

## 8. What frame a *new* GT source must be supplied in (2026-09-19)

Question asked directly: what body-frame convention must externally supplied ground truth use, e.g. to compare against a live SITL flight, given Basalt's `T_w_i` output? Not "front-right-up", as one might guess from ROS's usual body convention.

The body-frame axis convention is fixed by whichever IMU Basalt is actually integrating, not by any Basalt-internal choice, since no rotation is applied between the subscribed IMU topic and the estimator (`context/ap_dds_imu_stream.md` §"The orientation field, and why it is unused"). On the `djinn start sitl2` stack that IMU is ArduPilot's AP_DDS `/ap/imu/experimental/data`, confirmed NED (X-forward, Y-right, Z-down) by both `frame_id: base_link_ned` and a stationary specific force of `(−0.06, −0.01, −9.87)`. So GT orientation must be expressed in the **same NED body axes**, not FRU and not the FLU that Project AirSim's own (unused, bridged) IMU topic would give.

This connects directly to `/ws/context/project-airsim-architecture.md` §C.10/§C.5: Project AirSim's ROS 2 C++ bridge publishes `/actual_pose` in **NWU** (negate Y, negate Z from native NED, a 180° rotation about X), not NED and not true ENU. That conversion is an involution, so recovering NED from a bridged `/actual_pose` sample is the same negate-Y/negate-Z operation applied a second time. Feeding `/actual_pose` into Basalt as GT without this correction would silently hand it the FLU-family convention this section's sibling document (`airsim_camera_extrinsics.md` §"The one path on which the frame is not NED") already flags as the one place in the stack that is not NED, applied to the wrong topic besides.

The world-frame requirement from §3/§5b above still applies unchanged: GT's world origin and orientation need not match Basalt's gravity-aligned world, first-pose SE(3) alignment handles that. Only the per-sample body-axis convention and the "T_world_IMU, not T_world_camera" requirement are load-bearing for a new GT source.

A live ROS ground-truth subscription now exists in `BasaltSLAMNode`, selected by the `gt_pose_topic_name` and `gt_pose_type_name` parameters and described in §10. The offline EuRoC/TUM-VI-format CSV path of §5 remains the other source feeding the `GtPose` queue the visualiser consumes. Correction, 2026-09-25. This paragraph previously stated, as of 2026-09-19, that no live subscription existed (then confirmed in `context/vio_optical_flow_instrumentation.md`) and that a live feed would be new wiring. That wiring was added on 2026-09-25 by `/ws/ros_ws/plans/gt_ingestion.md`.

Full three-way frame contract, this SLAM world/body convention, what `/tf` actually publishes today (a defect, not just a convention, `T_i_w` mislabelled `"camera"`), what `/ap/tf`'s `AP_DDS_External_Odom` requires (`"odom"`/`"base_link"`, un-inverted FLU, REP-105 no-reset semantics), the single recurring NED↔FLU/NWU transform found at four sites, and per-source target frames for Project AirSim `/actual_pose` (NWU), ArduPilot's own ENU pose, and WGS-84 GPS, worked out in [`/ws/context/slam-ardupilot-tf-frame-contract.md`](/ws/context/slam-ardupilot-tf-frame-contract.md).

## 9. VO (no IMU) never gravity-aligns at all, `T_w_i_init` stays identity (2026-09-19)

Everything in §5c–§5e above is specific to the VIO path. In VO (`SlamMode::VO`, `eSLAMType::VSLAM`, which is `BasaltSLAMNode`'s **default** `slam_type`, `node.cpp:29`, VIO requires explicitly configuring `"VISLAM"`), no gravity alignment happens at all.

`Controller::initialize` always calls the two-argument `vio_estimator_->initialize(bg, ba)` in this app, because `BasaltSLAM::InitialiseSlam` (`slam.cpp:58-59`) always passes `t_ns=0`, `T_w_i=Sophus::SE3d()` (identity), `vel_w_i=Zero`, which is exactly the condition `controller.cpp:174` tests to route to that overload, for both VO and VIO alike (`vio_estimator_` is the polymorphic base, selected by `use_imu` at construction, `controller.cpp:165-170`). For VIO that overload defers to `FromTwoVectors` on the first IMU sample (§5c). For VO, `SqrtKeypointVoEstimator::initialize(bg, ba)` (`sqrt_keypoint_vo.cpp:121-171`) does no such thing, it only sets up threading state (`add_pose=false`, `prev_frame=nullptr`, etc.) and never touches `T_w_i_init`. `T_w_i_init` is a plain `SE3` member (`sqrt_keypoint_vo.h:244`) with no other assignment anywhere on the VO path, so it keeps Sophus's default constructor value, identity, for the estimator's whole life (confirmed by `getT_w_i_init()` at `sqrt_keypoint_vo.h:201-203` simply casting that never-touched member).

Consequence: VO's world frame is identity relative to the body/calibration frame at t=0, whatever physical orientation the vehicle happened to have when tracking started, with **zero correction toward gravity**. If the vehicle is level at start, as is normal, world Z equals body Z, which on this NED-calibrated platform is **down**, not up. VO's world "up" axis is not up, it is an uncorrected artefact of the calibration's body-frame convention. There is no equivalent of the §5e yaw-arbitrariness table for VO because there is no rotation at all to characterise, `T_w_i_init` is exactly `I` every run, not approximately, not variably.

## 10. Live ground truth ingestion and world seeding (2026-09-25)

Change record: `/ws/ros_ws/plans/gt_ingestion.md`. Every live ground truth source is converted by `GeodeticConverter` (`ros_ws/src/slam/include/slam/utils/frame_conversion.hpp`) into one canonical form, an ENU world with an FRD body. FRD is the NED body axes of §8, so the orientation compares directly with `T_w_i`. The conversion is `R = W · R_src · B`, `p = W · p_src`, with `W` the world map to ENU and `B = diag(1, −1, −1)` for sources whose body is FLU.

| Type | Source world, body | W | B |
|------|--------------------|---|---|
| enu (`/ap/pose/filtered`) | ENU, FLU | I | F |
| ned | NED, FRD | `[[0,1,0],[1,0,0],[0,0,−1]]` | I |
| nwu (`/actual_pose`) | NWU, FLU | `[[0,−1,0],[1,0,0],[0,0,1]]` | F |
| frd | FRD, FRD (D assumed along gravity) | as ned | I |
| gps (`/ap/navsat` + AP_DDS Imu) | WGS-84, NED/FRD attitude | as ned | I |
| geopose (`/ap/geopose/filtered`) | WGS-84, ENU/FLU | I | F |

`/ap/pose/filtered` is ENU world with FLU body because AP_DDS composes a 90 degree yaw after its NED to ENU quaternion swap (`AP_DDS_Client.cpp:469-477`). The AP_DDS Imu quaternion is written with permuted fields, see `ap_dds_imu_stream.md`. A standalone test on 2026-09-25 confirmed that the AP_DDS ENU/FLU pose, the AirSim NWU/FLU pose, the permuted IMU quaternion and GeoPose all reduce to the same ENU/FRD pose as the NED conversion.

The first ground truth sample now defines the SLAM world whenever a ground truth type is configured, which replaces the per-run first-pose alignment of §3 for the live path. `Controller::TrackMonocular` drops frames, draining `imu_data_queue`, until one carries ground truth. VIO then left multiplies its `FromTwoVectors` attitude by `Rz(yaw_gt − yaw_acc)`, which fixes world Z so the gravity alignment of §5c survives while the arbitrary heading of §5e becomes the ground truth heading, and sets the translation to the ground truth position. VO (§9) takes the whole ground truth pose as `T_w_i_init`, since it has no gravity reference of its own and the ground truth attitude is its only source of up. With `gt_pose_type_name` `none` nothing changes, and VO keeps the identity start of §9. The visualiser draws aligned ground truth without the `slam_first` product (`Controller::IsGroundTruthAligned`). Each frame writes a `gt_eval.csv` row, `t_ns, p_w_i, q_w_i, p_gt, q_gt`, with blank cells for an absent pose and quaternions in Eigen `(x, y, z, w)` order.
