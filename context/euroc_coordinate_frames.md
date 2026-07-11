# EuRoC Dataset Coordinate Frame Reference

**Date**: 2026-07-09
**Source**: Local sensor.yaml files + EuRoC paper analysis

---

## Frame Notation (ETH/ASL convention)

`T_AB` = transformation taking a point from frame B to frame A: `p_A = T_AB * p_B`

`p_XY_Z` = position of frame Y origin, expressed in frame Z (with respect to frame X)

`q_XY` = quaternion encoding the rotation part of `T_XY` (from Y to X)

---

## EuRoC Frames

### Body Frame (B)
- **Definition**: Rigidly attached to the sensor payload
- **Aligned with**: IMU sensor frame (`T_BS = Identity` for `imu0`)
- **T_BS for IMU**: Identity → Body frame IS the IMU frame

### Reference Frame (R) = World Frame
- **Definition**: Fixed inertial frame (Vicon/Leica lab frame)
- **Origin**: Physical origin of the motion capture system
- **Gravity aligned**: Yes (Z-up approximately)

### Camera Frames (S for cam0, cam1)
- **T_BS for cam0** (from sensor.yaml MH_01_easy):
  ```
  T_BS = [ 0.0149  -0.9999   0.0041  -0.0216 ]
         [ 0.9996   0.0150   0.0257  -0.0647 ]
         [-0.0258   0.0038   0.9997   0.0098 ]
         [ 0.0000   0.0000   0.0000   1.0000 ]
  ```
  This is `T_body_cam0`, i.e., how to transform a point from camera to body frame.
  In Basalt notation: `T_i_c[0]` (stored as T_imu_cam)

---

## Ground Truth Data File

**Path**: `mav0/state_groundtruth_estimate0/data.csv`

**Column meanings**:
```
timestamp [ns]
p_RS_R_x [m]  → position X of body in world frame
p_RS_R_y [m]  → position Y of body in world frame
p_RS_R_z [m]  → position Z of body in world frame
q_RS_w        → quaternion w (T_world_body rotation)
q_RS_x        → quaternion x
q_RS_y        → quaternion y
q_RS_z        → quaternion z
v_RS_R_x      → velocity X in world frame
... (biases omitted)
```

**Interpretation**: `(p_RS_R, q_RS)` = `T_world_IMU` = `T_w_i` in Basalt notation

**How it's parsed** in `dataset_io_euroc.h:read_gt_data_state()`:
```cpp
ss >> timestamp >> tmp >> pos[0] >> tmp >> pos[1] >> tmp >> pos[2] >>
    tmp >> q.w() >> tmp >> q.x() >> tmp >> q.y() >> tmp >> q.z() >> ...
data->gt_pose_data.emplace_back(q, pos);  // Sophus::SE3d(q, pos)
```
Sophus::SE3d(q, pos) constructs T with rotation q and translation pos → `T_w_i` ✓

---

## Basalt Calibration JSON (euroc_ds_calib.json)

The Basalt calibration stores:
- `T_imu_cam`: Transform from camera to IMU frame (= `T_BS` from cam sensor.yaml)
- `T_mocap_world`: Transform between Mocap world and SLAM world (usually Identity)
- `T_imu_marker`: Transform from MoCap marker to IMU (for `mocap0` GT source)

For `state_groundtruth_estimate0`:
- `T_mocap_world = Identity` (same world)
- `T_imu_marker = Identity` (GT is already body frame)
- No extra transform needed

---

## MH_01_easy: Initial GT Pose

The first GT sample shows:
```
position:   (4.688, -1.787, 0.783) meters from Vicon origin
quaternion: w=0.534, x=-0.153, y=-0.827, z=-0.082
```

This means the robot starts 4.7m from the lab origin with a yaw of approximately 180° (large y-component of quaternion). The SLAM world frame starts at (0,0,0) with identity rotation. This explains the large visual separation in the screenshot.

---

## Key Insight for Implementation

When comparing/visualising GT vs SLAM:
1. **Same sensor frame** ✓ — both are T_w_imu
2. **Different world origins** ✗ — needs first-pose alignment
3. **Fix**: `T_gt_aligned[i] = T_gt[0].inverse() * T_gt[i]`
