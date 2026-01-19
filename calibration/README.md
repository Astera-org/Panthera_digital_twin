# Eye-to-Hand Calibration

Estimates the rigid transformation from a static camera base to robot base.

## Goal

Record the robot end effector (ee) position in both coordinate systems:
- `P_camera = (u, v, w)` - ee position in camera frame (from stereo depth)
- `P_robot = (x, y, z)` - ee position in robot frame (from forward kinematics)

They are the **same physical point** in different coordinate systems.

Find `R` (rotation) and `t` (translation) such that:
```
P_robot = R @ P_camera + t
```

## Files
ee: end effector

| File | Purpose |
|------|---------|
| `calibrate_hand_eye.py` | Computes transformation from recorded ee pairs |
| `calibration_pairs_*.yaml` | Recorded (camera, robot) point pairs |
| `transform_*.yaml` | Output transformation matrix |
| `../backend/camera/cam2bot.yaml` | Active transformation used by backend |

## Workflow

### 1. Record Calibration Pairs

In the Digital Twin frontend:
1. Connect to robot backend
2. Open Camera panel, start camera
3. Click **Select** mode
4. Move robot end effector to a point visible in camera
5. Click on the ee point in the camera view
(you should see the ee coordinates in robot frame and camera frame)
6. Click **Record** to save the pair
7. Repeat for 30+ points across the workspace
8. Click **Save** to export to `calibration_pairs_*.yaml`

### 2. Compute Transformation

```bash
cd calibration
python calibrate_hand_eye.py calibration_pairs_YYYYMMDD_HHMMSS.yaml --save
```

Options:
- `--save` - Save transformation to `transform_*.yaml`
- `--k-fold 5` - Use 5-fold cross-validation
- `--test-ratio 0.3` - Use 30% hold-out validation

### 3. Deploy

Copy the transform to the camera folder:
```bash
cp transform_*.yaml ../backend/camera/cam2bot.yaml
```

Restart the backend. The **C2B** row will now appear in Select mode showing transformed coordinates.

## Output Format

```yaml
camera_to_robot_transform:
  rotation_matrix:
  - [r11, r12, r13]
  - [r21, r22, r23]
  - [r31, r32, r33]
  translation:
  - tx
  - ty
  - tz
```

Usage: `P_robot = R @ P_camera + t`
