# Helmet Mocap Dataset: Algorithm Usage Guide

## 1. Dataset summary

This dataset is a UE5-generated, synchronized, four-camera rigid-body sequence.

| Item | Value |
|---|---:|
| Rigid body | `helmet_01` |
| Markers | `H01` to `H06` |
| Cameras | `Camera_01` to `Camera_04` |
| Resolution | 1280 x 720 |
| Frame rate | 30 FPS |
| Frame count | 90 per camera |
| Duration | 3.0 s |
| Horizontal FOV | 70 degrees |
| Camera-to-body range | 1.245 to 1.934 m |
| Length unit in JSON | cm |
| Image coordinate origin | top-left |

All camera streams are synchronized by `frame_index`. For example,
`Camera_01/frame_0020.png` and `Camera_04/frame_0020.png` represent the same
simulation time.

## 2. Directory contents

```text
dataset_helmet_0001/
|-- images/
|   |-- Camera_01/frame_0000.png ... frame_0089.png
|   |-- Camera_02/frame_0000.png ... frame_0089.png
|   |-- Camera_03/frame_0000.png ... frame_0089.png
|   `-- Camera_04/frame_0000.png ... frame_0089.png
|-- videos/
|   |-- Camera_01.mp4 ... Camera_04.mp4
|   `-- Camera_01.gif ... Camera_04.gif
|-- camera.json
|-- rigid_bodies.json
|-- object_pose_frames.json
|-- gt_3d_frames.json
|-- gt_2d_frames.json
|-- visibility.json
|-- scene_config.json
`-- ALGORITHM_USAGE_GUIDE.md
```

- `images/`: primary algorithm input. PNG avoids video compression artifacts.
- `videos/`: preview or video-decoder integration input. Do not use MP4 for
  pixel-accurate image comparison when PNG is available.
- `camera.json`: camera intrinsics, extrinsics, coordinate conventions, and
  transformation matrices.
- `rigid_bodies.json`: rigid-body definition and the six marker positions in
  the helmet-local coordinate system.
- `object_pose_frames.json`: ground-truth helmet pose for every frame.
- `gt_3d_frames.json`: local and world 3D position of every marker per frame.
- `gt_2d_frames.json`: marker position in each camera and ideal 2D projection.
- `visibility.json`: in-front and image-bound checks for every marker.
- `scene_config.json`: sequence, trajectory, lighting, distance, and image
  generation settings.

## 3. Coordinate systems

### Unreal world coordinates

- `+X`: room length direction
- `+Y`: room width direction
- `+Z`: upward
- Unit: cm

### OpenCV camera coordinates

- `+X`: image right
- `+Y`: image down
- `+Z`: camera forward
- Unit: cm

Matrices are stored in row-major order and operate on homogeneous column
vectors:

```text
p_camera = T_world_to_camera_opencv * p_world
p_world  = T_camera_opencv_to_world * p_camera
```

The object pose uses:

```text
p_world = T_object_to_world * p_object
```

Translation fields are in cm. Convert to metres with `value_m = value_cm / 100`.
Quaternions use `[x, y, z, w]`, not `[w, x, y, z]`.

## 4. Camera parameters

Each `camera.json/cameras[]` entry contains:

- `camera_id`: stable camera name.
- `intrinsics.K`: 3 x 3 intrinsic matrix.
- `intrinsics.fx`, `fy`, `cx`, `cy`: pinhole parameters in pixels.
- `intrinsics.width`, `height`: image dimensions.
- `intrinsics.distortion_coefficients`: currently all zero.
- `extrinsics.T_world_to_camera_opencv`: world-to-camera transform.
- `extrinsics.T_camera_opencv_to_world`: inverse transform.
- `extrinsics.ue_world_position_cm`: camera location in UE world coordinates.

The ideal projection is:

```text
u = fx * Xc / Zc + cx
v = fy * Yc / Zc + cy
```

where `[Xc, Yc, Zc]` is `camera_xyz_cm`. A point is geometrically valid when
`Zc > 0`, `0 <= u < 1280`, and `0 <= v < 720`.

## 5. Rigid-body and marker data

`rigid_bodies.json` defines marker coordinates relative to the helmet origin:

```text
rigid_bodies[0].markers[i].id
rigid_bodies[0].markers[i].position_cm
```

Marker IDs are unique and stable: `H01`, `H02`, `H03`, `H04`, `H05`, `H06`.
Do not infer identity from left-to-right image order because that order changes
with pose and camera viewpoint.

For frame `f`, the world position expected from the rigid-body pose is:

```text
p_world_gt = T_object_to_world[f] * p_marker_local
```

It can be compared directly with
`gt_3d_frames.json/frames[f].markers[].world_position_cm`.

## 6. Frame-level ground truth

### Helmet pose

`object_pose_frames.json` provides:

- `frame_index` and `time_sec`
- `location_cm`
- `rotation_pitch_yaw_roll_deg`
- `quaternion_xyzw`
- `T_object_to_world`

For pose evaluation, prefer `T_object_to_world` or `quaternion_xyzw` over Euler
angles. Euler angles are included mainly for inspection.

### Marker 3D ground truth

`gt_3d_frames.json` provides, for each marker:

- `local_position_cm`: fixed coordinate in the helmet frame
- `world_position_cm`: marker coordinate after applying the frame pose

### Marker 2D ground truth

`gt_2d_frames.json` provides, for every camera and marker:

- `camera_xyz_cm`: OpenCV camera-space coordinate
- `pixel_uv`: ideal projected pixel `[u, v]`

The PNG corresponding to frame `f` is:

```text
images/<camera_id>/frame_<f as four digits>.png
```

## 7. Visibility semantics

`visibility.json` contains:

- `in_front`: marker has positive camera depth.
- `in_frame`: projected pixel lies inside the image rectangle.
- `occlusion_tested`: currently `false`.

Important: `in_frame=true` does not guarantee that the marker is visually
unoccluded by the helmet. This version does not perform a physical ray-cast
occlusion test. Detection evaluation should either inspect the image or ignore
markers known to be physically hidden.

## 8. Recommended algorithm workflow

1. Load `scene_config.json`, `camera.json`, and `rigid_bodies.json` once.
2. Read synchronized PNG frames with the same `frame_index` from all cameras.
3. Detect red marker centres in each image.
4. Associate detections with `H01` to `H06`, or solve correspondence jointly
   using the known rigid marker geometry.
5. Use two or more camera observations and the exported projection matrices to
   triangulate marker 3D positions.
6. Estimate helmet pose by aligning local marker coordinates to reconstructed
   world coordinates.
7. Compare estimated 2D points, 3D points, and pose against the JSON truth.

## 9. Evaluation metrics

### 2D reprojection error

For valid visible marker observations:

```text
e_2d_px = sqrt((u_est - u_gt)^2 + (v_est - v_gt)^2)
```

Report mean, median, 95th percentile, and per-camera error.

### 3D marker error

```text
e_3d_cm = norm(p_est_world_cm - p_gt_world_cm)
```

Report per-marker and per-frame statistics. Divide by 100 for metres.

### Position error

```text
e_position_cm = norm(t_est_cm - t_gt_cm)
```

### Rotation error

After normalizing both quaternions:

```text
e_rotation_deg = 2 * acos(clamp(abs(dot(q_est, q_gt)), 0, 1)) * 180 / pi
```

The absolute dot product handles the fact that `q` and `-q` represent the same
rotation.

## 10. Validation

Run from the project root before using or transferring the dataset:

```powershell
python validate_dataset_v054.py --dataset output\dataset_helmet_0001
```

Expected result:

```text
Dataset validation passed
  Cameras: 4
  Frames per camera: 90
  Total PNG frames: 360
  Markers per rigid body: 6
```

The validator checks file presence, continuous frame numbering, camera sets,
frame counts, six unique marker IDs, and consistency between rigid-body and
frame-level ground truth.

## 11. Current limitations

- Ideal pinhole camera; lens distortion coefficients are zero.
- No motion blur, sensor noise, rolling shutter, or exposure variation model.
- Visibility does not include physical occlusion testing.
- MP4 is compressed and intended primarily for preview/integration testing.
- This sequence is suitable for pipeline integration and first-stage accuracy
  evaluation, but it is not yet a final real-camera-equivalent benchmark.
