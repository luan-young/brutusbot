# Plan: Monocular visual SLAM for Brutusbot (UcoSLAM-first, ORB-SLAM3 fallback), integrated with Nav2

This is a living document. Run it one phase at a time. When a phase is done, update the status table. Record any change of direction in the decision log and edit the affected phase sections.

## Status

| Phase | Description | Status | Notes |
|---|---|---|---|
| 0 | Backend spike (UcoSLAM go/no-go) | Done | **Go for UcoSLAM** (2026-10-11): all 6 criteria pass; 0b fallback not needed |
| 1 | Camera + markers (sim and real) | In progress | 1a done (2026-10-12): hand-measured mount x 0.07, z 0.21, level camera. Next: 1b |
| 2 | `visual_slam_node` + `SlamBackend` | Not started | |
| 3 | Map frame, scale, `map → odom` | Not started | |
| 4 | Mapping mode, occupancy grid, save | Not started | |
| 5 | Navigation mode (localization only) | Not started | |
| 6 | Nav2 integration + bringup | Not started | |

Status values: Not started / In progress / Done / Blocked.

## Decision log

- 2026-10-03: Camera is **monocular**, `/image_raw` from `v4l2_camera`.
- 2026-10-03: v1 assumes a **fully static environment**. No dynamic-obstacle detection.
- 2026-10-03: Try **UcoSLAM + ArUco markers** first for metric scale. ORB-SLAM3 with odometry-based scale is the fallback. Phase 0 decides.
- 2026-10-10: UcoSLAM passes scale (≤ 1.3% error) and relocalization on the laptop. The go/no-go now depends only on Pi 5 performance.
- 2026-10-11: **Go for UcoSLAM.** On the Pi 5, with the room lit and the camera moving, localization tracked 100% of frames at 54.7 Hz median (target ≥ 10 Hz). The ORB-SLAM3 fallback (0b) is dropped unless a later phase hits a blocker.
- 2026-10-12: Phase 1 detailed.
  - The camera mount pose lives in `brutusbot_description/config/camera_mount.yaml`, relative to `base_footprint`, loaded by xacro. Hand-measured values are only the starting point. They're refined by an extrinsic calibration (1f: checkerboard on the floor + wheel odometry, stop-and-go), because the mount is hand-built.
  - The sim camera hfov is 0.85 rad to match the C270 (f ≈ 707 px), not ~1.3 rad.
  - The sim uses its own `worlds/empty.sdf`, because Gazebo's stock one has no sensors system.
  - Intrinsics are redone with ROS `camera_calibration`, since the Phase 0 distortion terms look over-fitted. `/camera_info` becomes the only intrinsics source.
- 2026-10-12: Hand-measured camera mount, relative to `base_footprint`: x 0.07 m, y 0, z 0.21 m, roll/pitch/yaw 0.
  - The camera is **level**, not pitched ~10° down as first assumed.
  - With a vertical FOV of ±18.8°, the floor is only visible from ~0.62 m ahead, so a checkerboard lying on the floor is seen at a grazing angle. 1f therefore uses an **upright board** standing on the floor by default; floor mode stays available.
  - Markers go on walls at roughly 0.2 m centre height, below most furniture tops, so check that furniture doesn't hide them.

## Context

`brutusbot_slam` is an empty scaffold on the `visual-slam` branch. The goal is a monocular visual SLAM stack using `/image_raw` from a `v4l2_camera` node that:
1. publishes the `map → odom` TF from the SLAM camera pose (instead of AMCL or slam_toolbox),
2. builds a 2D occupancy grid from SLAM map points in a **mapping** mode and saves it as a static map,
3. in **navigation** mode, relocalizes against the saved map (localization only, the map is not modified),
4. plugs into Nav2 for planning, control and behaviors.

**Scope for v1: the environment is fully static.** There is no dynamic-obstacle detection, and the saved map is Nav2's only obstacle source.

**The main problem is monocular scale.** Monocular SLAM only knows poses up to an unknown scale. The chosen direction is **UcoSLAM with printed ArUco markers**: a marker of known size gives a metric pose from a single frame. That fixes scale, makes initialization and relocalization reliable, and lets a chosen marker define a repeatable map frame. UcoSLAM is less mature than ORB-SLAM3, so a **go/no-go spike** comes first. **ORB-SLAM3 with scale estimated from wheel odometry** is the fallback. Everything after the backend sits behind a small `SlamBackend` interface, so most of the work is the same either way.

Current state:
- Installed: Nav2, slam_toolbox, v4l2_camera, ros_gz_image, OpenCV 4.6, Eigen 3.4.
- Not installed: UcoSLAM, ORB-SLAM3, Pangolin.
- The URDF has **no camera link** and no IMU.
- The Gazebo world is `empty.sdf`, which has no texture and no markers.

## Architecture

```
v4l2_camera ──/image_raw,/camera_info──► brutusbot_slam/visual_slam_node ──TF map→odom──► Nav2
diff_drive  ──TF odom→base_footprint──►        │   (SlamBackend: UcoSLAM | ORB-SLAM3)
robot_state_publisher ──TF base→camera──►      ├─ /map (OccupancyGrid, mapping mode)
                                               └─ /slam/pose, /slam/state
maps/<name>.<backend map> + <name>.yaml/.pgm (grid) + <name>_slam.yaml (scale, T_map_slam, backend, intrinsics)
```

Frames: `map → odom → base_footprint → base_link → camera_link → camera_optical_link`. The backend's world frame (`slam_world`) is internal and never published.

## Phase 0: Backend spike (go/no-go), 1–2 days, outside colcon

**0a. UcoSLAM spike**
- Build UcoSLAM in `~/ucoslam` on this laptop and on the Pi 5. Expect fixes for Ubuntu 24.04 / OpenCV 4.6 (C++17, deprecated OpenCV API). Check the license (GPLv3 vs commercial) and fix any build problems in `brutusbot_slam/third_party/ucoslam.patch`.
- Print ArUco markers from one dictionary (e.g. `ARUCO_MIP_36h12`, as UcoSLAM recommends) at ~15–20 cm. **Measure the printed size** and use the measured value.
- Use UcoSLAM's own monocular example with the USB camera, calibrated as in Phase 1, and a few markers on walls.
- **Go criteria** (record results in the plan or README):
  1. It builds and runs on the Pi 5 at ≥ 10 Hz at 640×480.
  2. Distance error over a measured 3 m straight line is < 3%.
  3. It relocalizes immediately when a known marker comes back into view after a kidnap.
  4. Save, load and localization-only mode all work.
  5. The map (keypoints, markers, keyframes) is reachable through the C++ API without heavy patching.
  6. The license is acceptable.
- Note how many markers per room are needed to keep scale stable between markers.

**0a results (updated as tests are run)**

Setup on the laptop (x86_64, 12 cores):
- UcoSLAM 1.1.1 is in `~/dev/ucoslam1.1.1`, built in `build/`.
- Data is in `~/dev/ucoslam_data`:
  - Logitech C270 calibration at 640×480: `camera_calibration/01/c270_640x480.yml`.
  - Vocabulary: `orb.fbow`.
  - Room map: `map01/world.map` (18 markers, `ARUCO_MIP_36h12`, measured size 8.135 cm).
  - Params: `map01/params_map.yml` (mapping, `runSequential: 1`) and `map01/params_loc.yml` (localization, `runSequential: 0`, `forceInitializationFromMarkers: 1`).
  - Camera exposure and white balance are fixed with `v4l2-ctl` (`comandos_gerais.txt`).

| # | Criterion | Result |
|---|---|---|
| 1 | Pi 5, ≥ 10 Hz at 640×480 | **Pass**, 2026-10-11. Built on the Pi 5 (4 GB, aarch64) with `third_party/ucoslam_aarch64.patch`: upstream only skips x86 SIMD/cpuid code on Android, and its CMake applies x86 flags to aarch64. Ran headless with `-noX`, `-loc_only`, `params_loc.yml` (4000 features) for 60 s at the camera's 30 fps. Rates are UcoSLAM processing time (30-frame rolling average). **Lit room, camera moved by hand across many markers: 2020 frames, 100% tracked, 54.7 Hz median / 50.3 Hz p10 / 48.5 Hz min.** Max 49 °C, no throttling. (An earlier run in a dark room with the camera still tracked only 27%, at 68.5 Hz while tracking.) Camera controls from `~/dev/ucoslam_data/c270_settings.txt` stayed set when UcoSLAM opened the camera, because OpenCV didn't reset them. Not yet measured: mapping-mode speed on the Pi (only matters if maps are built on the Pi). |
| 2 | Scale error < 3% over ~3 m | **Pass** (2026-10-10). Errors are +0.2% to +1.3% over 1.29–2.06 m (table below). Pairs longer than 2.06 m weren't measured. |
| 3 | Immediate relocalization after a kidnap | **Pass** (2026-10-10). Relocalizes as soon as a known marker comes back into view. |
| 4 | Save, load and localization-only mode | **Pass**. The map is saved, then loaded with `-loc_only` + `params_loc.yml`; the relocalization test ran in this mode. |
| 5 | Map reachable through the C++ API | **Pass**, no patch needed. `UcoSlam::getMap()` returns `shared_ptr<Map>` with public `map_points` (`getCoordinates()`, `isBad()`, `getNumOfObservingFrames()`, `getObservingFrames()`), `map_markers` (`get3DPoints()`) and `keyframes` (`getCameraCenter()`). Mode switching uses `setMode(MODE_LOCALIZATION)`, and maps use `saveToFile`/`readFromFile`. Caveat: with `runSequential: 0`, mapping runs in a background thread, so the grid builder has to snapshot the map safely (after `waitForFinished()` or under a lock). |
| 6 | License | **Acceptable for a personal/open project.** UcoSLAM is GPLv3, and the repo is MIT. Our MIT source can link it, but any distributed binary of `brutusbot_slam` falls under GPLv3. Set `brutusbot_slam/package.xml` license to `GPLv3`. |

Map sanity check (`markermap.yml`):
- All 18 markers come out at 8.13–8.14 cm.
- The four walls are consistent: front wall ids 0–5, side wall ids 6–10, back wall ids 11–16, other side wall id 17. Front and back are parallel, and the sides are square to them.
- The map frame is the first camera's frame (y roughly down), tilted about 3–6° from gravity. Phase 3's map-frame definition must handle this.

Distances to check with a tape measure (marker centre to centre; scale error = map / tape − 1):

| Pair | Map distance | Tape distance | Scale error |
|---|---|---|---|
| 0–5 (same wall) | 1.305 m | 1.288 | 0.013 |
| 6–10 (same wall) | 1.377 m | 1.359 | 0.013 |
| 11–16 (same wall) | 1.735 m | 1.719 | 0.009 |
| 3–13 (across the room) | 2.497 m | not measured | |
| 6–17 (diagonal) | 3.813 m | not measured | |
| Front wall to back wall (perpendicular) | 2.060 m | 2.055 | 0.002 |
| Side wall (6–10) to the marker 17 wall | 3.478 m | not measured | |

Every error is positive: the map is consistently about 1% larger than the room. This systematic bias most likely comes from the marker size (1% of 81 mm is only about 0.8 mm), or possibly from the calibrated focal length. Optional fix: re-measure the printed markers with calipers, or set `aruco_markerSize` to about 0.0805, then remap and check again. It isn't needed to pass.

The markers are small (8.1 cm). At 640×480 (f ≈ 707 px), a marker 3 m away is only about 19 px wide, so marker poses are noisy beyond roughly 2–2.5 m. If criterion 2 or 3 is marginal, try larger markers (15–20 cm) before giving up on UcoSLAM.

**0b. Fallback (only if UcoSLAM fails):** build Pangolin v0.9 + ORB-SLAM3 in `~/ORB_SLAM3` (C++17, relax the OpenCV pin, same `-march` as our node). Add `third_party/orb_slam3.patch` with public `GetAtlas()` and `SaveAtlas()`. Smoke-test with `mono_tum`.

Add `brutusbot_slam/cmake/Find<Backend>.cmake`, driven by a `<BACKEND>_ROOT` cache/env var. Only the chosen backend is a build dependency.

## Phase 1: Camera and markers in sim and on the real robot

Goal: the camera exists in the URDF at a **calibrated** pose, publishes `/image_raw` + `/camera_info` in sim and on the real robot, and the sim has a room with ArUco markers. Nothing in this phase depends on UcoSLAM.

Work in this order: 1a → 1b → 1c (sim, can run on the laptop alone), then 1d → 1e → 1f (real robot), then 1g. 1f is tested in sim before it's used on the real robot.

### 1a. Camera mount in the URDF (nominal pose)

- **Mount values file** `brutusbot_description/config/camera_mount.yaml`. It is the single source of truth for `T_base_footprint_camera_link`, and the 1f calibration tool overwrites it:
  ```yaml
  # source: nominal (hand-measured) | calibrated YYYY-MM-DD (rms ..., std ...)
  parent: base_footprint     # the values below are relative to base_footprint
  x: 0.0      # m, forward from the wheel axle midpoint
  y: 0.0      # m, left
  z: 0.0      # m, lens centre height above the floor
  roll: 0.0   # rad
  pitch: 0.0  # rad, positive = looking down
  yaw: 0.0    # rad, positive = looking left
  ```
  Values are relative to `base_footprint` (the floor point below the wheel axle midpoint), not `base_link`, because that's what can be measured on the robot with a tape and what 1f estimates. The URDF box sizes are rough, so the `base_link` offset is a model detail that shouldn't leak into the calibration.
- **Nominal values (hand-measured 2026-10-12)**: x 0.07 m, y 0, z 0.21 m, roll/pitch/yaw 0 (level camera). They're measured from the floor point midway between the two wheel contact points to the lens centre; ±2 cm and ±5° is good enough, since 1f refines them.
- **`urdf/camera.xacro`**, included from `brutusbot.xacro` for both sim and real:
  - Load the file with `xacro.load_yaml('$(find brutusbot_description)/config/camera_mount.yaml')`.
  - `camera_joint` (fixed), parent `base_link`, child `camera_link`. Origin = mount values minus the `base_joint` offset (`${-motor_position} 0 ${wheel_radius + motor_height / 2}`). `base_joint` is a pure translation, so the subtraction is exact. Move those two expressions into properties shared with `mobile_base.xacro` so they can't drift apart.
  - `camera_link` is the body frame: x along the optical axis, z up. It gets a small box visual and collision (C270 body ≈ 0.03 × 0.08 × 0.03 m) and a light inertial so Gazebo keeps the link.
  - `camera_optical_link` with a fixed joint, rpy `-π/2 0 -π/2` (z forward, x right, y down).
- `CMakeLists.txt` of `brutusbot_description`: install `config/` (and `worlds/`, `models/` from 1c).
- **Check**: `display.launch.py` shows the camera at the expected spot. `ros2 run tf2_ros tf2_echo base_footprint camera_optical_link` matches the YAML (translation as given, optical z axis pointing forward and down by `pitch`).

### 1b. Sim camera

- In `brutusbot_gazebo.xacro`, `<gazebo reference="camera_link">` with a gz `camera` sensor:
  - 640×480, 30 Hz, `R8G8B8`. **hfov = 0.85 rad**, which matches the C270's f ≈ 707 px at 640 px (`2·atan(320/707)`), so the sim and the real images look alike. (The earlier draft said ~1.3 rad, which is far wider than the real camera.)
  - clip 0.05–20 m, gaussian noise stddev ~0.007, no distortion.
  - `<topic>image_raw</topic>` (gz publishes `camera_info` next to it), `<gz_frame_id>camera_optical_link</gz_frame_id>`, so the image header carries the optical frame like on the real robot.
- **Sensors system**: Gazebo's stock `empty.sdf` doesn't load `gz-sim-sensors-system`, so no camera would render. Add our own `worlds/empty.sdf` (stock world + `gz-sim-sensors-system` with `ogre2`) and use it as the default world. Today's behavior is unchanged apart from the camera.
- **Bridge** in `gazebo.launch.py`: `ros_gz_image image_bridge /image_raw` (lighter than `parameter_bridge` for images) and `/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo` added to the existing `parameter_bridge`.
- **Ground truth for later phases**: add the `gz-sim-pose-publisher-system` to the robot (model pose only) and bridge it to `/ground_truth/pose`. Phases 3–5 compare `/slam/pose` against it. It costs nothing to add now.
- **Check**: `ros2 topic hz /image_raw` ≈ 30 Hz with `use_sim_time`, `ros2 run rqt_image_view rqt_image_view` shows the scene, `/camera_info` K has fx ≈ 707.

### 1c. Textured room with ArUco markers

- **Marker models**: script `brutusbot_description/scripts/make_marker_models.py`. It takes the ARUCO_MIP_36h12 PNGs (from `~/dev/ucoslam_data/markers/aruco_mip_36h12_dict.zip`), a list of ids and a size, and writes `models/aruco_<id>/` (`model.config`, `model.sdf` with a 2 mm box, PBR `albedo_map` texture on the front face, white quiet zone around the black border). Run it once and commit the generated models. They're small.
- **Marker size**: sim uses **0.15 m** (black-border edge). Phase 0 showed that 8.1 cm markers get noisy beyond ~2 m, and the robot sees them from further than the hand-held tests did. The size is a per-setup parameter (Phase 2 `config/ucoslam.yaml`), so sim and real don't have to match.
- **`worlds/textured_room.sdf`**: ~5 × 4 m room, walls 1 m high.
  - Walls and floor get **non-repeating** photo textures (e.g. CC0 textures from ambientCG/Poly Haven, committed under `models/room_textures/`). Plain-colour or tiled walls give no ORB features or repeated ones.
  - 3–4 boxes or furniture shapes for obstacles and parallax.
  - ~12 markers on walls and boxes, centres at camera height ± 0.1 m, roughly one every 1.5–2 m along the walls, with id 0 visible from the spawn pose (it's Phase 3's `origin_marker_id`).
  - A **checkerboard model** (`models/calib_checkerboard/`, same pattern as the real one from 1f) standing upright on the floor ~1 m in front of the spawn pose, for testing 1f in sim. Put it to one side of the driving area, or give it its own small world, so it doesn't block driving.
  - Lights: one directional + ambient, no hard shadows over markers.
- **Ground-truth marker poses** in `worlds/textured_room_markers.yaml` (id, size, pose in the world frame), written by hand next to the SDF. Phase 3's tests compare the map frame against it.
- `gazebo.launch.py`: add a `world` launch arg (`empty` | `textured_room`, default `empty`), resolved to `brutusbot_description/worlds/<world>.sdf`. `GZ_SIM_RESOURCE_PATH` must also include `brutusbot_description/models`.
- **Check**: in `textured_room`, every marker is detected by `aruco_test` from `~/dev/aruco-3.1.12` on sim frames saved with `ros2 run image_view image_saver`, at 1 m and 3 m.

### 1d. Real camera node (on the Pi 5)

- `brutusbot_bringup/config/v4l2_camera.yaml`:
  - `video_device: /dev/video0` (check with `v4l2-ctl --list-devices`; consider a udev symlink `/dev/brutusbot_camera`),
  - `image_size: [640, 480]`, `pixel_format: YUYV`, `output_encoding: rgb8`, `camera_frame_id: camera_optical_link`, `camera_name: c270`,
  - `camera_info_url: package://brutusbot_bringup/config/camera_calibration.yaml`,
  - the fixed controls from `~/dev/ucoslam_data/c270_settings.txt` (manual exposure 150, gain 100, white balance 6000 K, power-line 50 Hz, ...). `v4l2_camera` exposes V4L2 controls as parameters. Confirm the exact parameter names with `ros2 param list /v4l2_camera` on the Pi first, and check that `auto_exposure` is applied before `exposure_time_absolute` (if the order is wrong, set the controls with `v4l2-ctl` in a launch `ExecuteProcess` before the node starts instead).
- `real_robot.launch.py`: add a `camera` launch arg (default `true`) that starts `v4l2_camera_node` with that file. Add `v4l2_camera` to `brutusbot_bringup/package.xml` and install it on the Pi.
- **Check (on the Pi)**: `ros2 topic hz /image_raw` ≈ 30 Hz, `ros2 topic echo /camera_info --once` has the calibrated K, `v4l2-ctl -C exposure_time_absolute` shows the fixed value. Don't view raw images over Wi-Fi to judge the frame rate (~27 MB/s); measure on the Pi.

### 1e. Intrinsic calibration

- The Phase 0 calibration (`c270_640x480.yml`, aruco_calibration) has distortion `k2 = 1.07`, `k3 = -3.6`. Values that large, with opposite signs, usually mean an over-fit from too little coverage of the image corners. It still worked for Phase 0, but it should be redone in ROS format.
- `sudo apt install ros-jazzy-camera-calibration` (laptop; the GUI needs a display). Print a checkerboard on a **rigid** board (A3 if possible), measure the square size with calipers, then, with the camera node running on the Pi:
  `ros2 run camera_calibration cameracalibrator --size <cols>x<rows> --square <m> --no-service-check image:=/image_raw camera:=/c270`
  Fill all four bars (X, Y, size, skew), with the board in the image corners and tilted.
- Target: reprojection error < 0.3 px, |k2|, |k3| well below 1. Save the result as `brutusbot_bringup/config/camera_calibration.yaml` (`camera_name: c270`).
- **Compare fx/fy with the Phase 0 value (707)**. Phase 0 maps came out ~1% too large. If the new focal length is ~1% lower, that was the cause, and it gets recorded in the decision log. Then delete the old `.yml` from the workflow: from Phase 2 on, the node builds UcoSLAM's camera parameters from `/camera_info`, so this file is the only intrinsics source.

### 1f. Extrinsic calibration: `base_footprint → camera_link`

The mount is hand-built, so the hand measurements from 1a are only a starting point. The extrinsic matters because Phase 3 computes the robot pose as `T_map_base = T_map_slam · T_slam_cam · T_cam_base`:
- a **yaw** error makes Nav2 see the robot crab sideways (≈ 1.7 cm per metre per degree), and the controller fights it;
- a **height / pitch / roll** error shifts the occupancy-grid height band of Phase 4 (2° of pitch ≈ 10 cm at 3 m) and the floor projection of Phase 3;
- an **x / y** error shows up as the pose wobbling during in-place rotations.

**Targets**: z ±1 cm, roll/pitch ±0.5°, yaw ±1°, x/y ±2 cm.

**Method: fixed checkerboard + wheel odometry, one session, stop-and-go.**
- A rigid checkerboard (the 1e board is fine) **stays put** for the whole session. It can be placed two ways (`--board-mode`):
  - **`upright` (default for this level camera at 0.21 m)**: the board stands vertically on the floor, facing the robot ~0.8–1.5 m away, square to the floor (check with a set square and clamp it to a box or the wall). Measure the height of the bottom corner row above the floor with a ruler (±1 mm) and pass it as `--board-height`.
  - **`floor`**: the board lies flat on the floor. This only makes sense if the camera is pitched down. With the current mount, the floor is out of view closer than ~0.62 m, and further away the board is seen at a grazing angle.
- A chessboard is used instead of ArUco markers because OpenCV 4.6's `cv2.aruco` has no `ARUCO_MIP_36h12`, and corner-based PnP is more accurate anyway.
- The robot is driven with the joystick through ~20 stationary poses, keeping the board in view (hfov ±24°): in-place rotations of about ±8° and ±15°, forward/backward steps of 0.2–0.5 m, and a few combinations. At each pose it stops for ~1 s.
- At each stationary pose, the tool averages a few frames of `T_cam_board` (`findChessboardCornersSB` + `solvePnP`, using `/camera_info`) and reads `T_odom_base_footprint` from TF at the image stamps. "Stationary" is detected automatically (odometry twist ≈ 0 for 0.5 s), so the user only drives. Stop-and-go avoids motion blur and camera/odometry time-sync errors.
- **Why it works (observability)**:
  - The board's orientation relative to the floor is known (vertical, or flat), and in upright mode so is its height. Every view therefore gives the camera's **height, roll and pitch**, whatever the board's position on the floor. In upright mode this relies on the board being truly vertical (1° of lean ≈ 1° of pitch error); in floor mode it relies on the floor being flat.
  - Rotating in place moves the camera on a circle around the robot's real rotation axis → **x, y** relative to the wheel axle midpoint, which is exactly `base_footprint`.
  - Driving straight shows the camera which way "forward" is → **yaw**.
  - Hand measurements alone can't give these to the targets above, and the board's position on the floor never has to be measured.
- **Solve**: nonlinear least squares (`scipy.optimize.least_squares`, Huber loss) over the 6-DoF `T_base_cam` plus the board's planar pose in `odom` (x, y, θ). Its height and tilt are fixed by the mode: upright → bottom corner row at `--board-height`, board plane vertical; floor → z = board thickness, flat, minimizing the board corner reprojection error over all poses, with `T_odom_base` from odometry. The 1a nominal values are the initial guess.
- **Output**:
  - The estimate, per-axis standard deviation (from the Jacobian), RMS reprojection error, and the difference from the nominal values. It warns if any std is above target or the result is more than 5 cm / 10° from nominal (wrong frame or sign).
  - **Odometry by-products, report only**: the ratio of camera-measured to odometry-measured rotation (wheel-separation error) and translation (wheel-radius error). Phase 3's scale sanity check depends on odometry, so a known bias here saves debugging later. Changing `brutusbot_controllers.yaml` is a separate decision.
  - With `--write <path>`, it rewrites `camera_mount.yaml` with the result and the `source: calibrated ...` header. With `--symlink-install` the change applies on the next launch.
- **Tool**: `brutusbot_slam/scripts/calibrate_camera_extrinsics.py` (rclpy, OpenCV, NumPy, SciPy; installed with `install(PROGRAMS ...)`). It's headless and prints progress on the console, so it can run on the Pi over SSH. It also publishes `/calibration/debug_image` with detected corners. Args: `--board <cols>x<rows> --square <m> [--board-mode upright|floor] [--board-height <m>] [--board-thickness <m>] [--write <path>]`. Needs `real_robot.launch.py` (or sim) running: controllers for the `odom` TF, and the camera.
- **Procedure**:
  1. **Sim first**: in `textured_room`, set the URDF mount to known values, start the tool with an initial guess offset by ~3 cm / 5° (a `--init` override), drive the poses. It must recover the URDF values within the targets. This validates the tool and the math before real data.
  2. **Real robot**: flat, rigid floor (no carpet), board vertical and well lit, fixed camera exposure (1d). Run once, write the YAML, then **run a second time** with the new values as the initial guess. The two runs must agree within the targets (repeatability check).
  3. Commit the YAML and record the result (values, std, odometry ratios) in the decision log.
- **Recalibrate** whenever the camera is touched or the mount is bent.

### 1g. Marker printing and layout guide (README)

- Print markers from `ARUCO_MIP_36h12` at 15–20 cm, on matte paper (no glare), glued flat to a rigid backing. **Measure the black-border edge with calipers** and use that value; Phase 0's +1% bias may have come from 0.8 mm of size error.
- Layout: at least one marker visible from the start pose, then one every ~2–3 m along driven routes and at goal areas, centres at roughly camera height (z from 1f). Avoid views steeper than ~60° from the marker normal. Never reuse an id in the same map.
- Keep a list of the ids used per room.

### Phase 1 done when

1. `display.launch.py` and `simulated_robot.launch.py` show `camera_link` / `camera_optical_link` in the right place, and the default sim behaves as before plus the camera.
2. Sim: `/image_raw` ≈ 30 Hz, `/camera_info` fx ≈ 707, all `textured_room` markers detectable at 1–3 m, `/ground_truth/pose` published.
3. Real: `/image_raw` ≈ 30 Hz measured on the Pi with fixed exposure; new intrinsics with < 0.3 px reprojection error, compared against Phase 0.
4. 1f recovers known extrinsics in sim within the targets; on the real robot, two runs agree within the targets; `camera_mount.yaml` committed with `source: calibrated`.
5. README has the camera, calibration and marker sections.

## Phase 2: `visual_slam_node` with a backend abstraction (C++, `brutusbot_slam`)

Files:
- `include/brutusbot_slam/{visual_slam_node,slam_backend,ucoslam_backend,scale_estimator,occupancy_grid_builder,pose_utils}.hpp`
- matching `src/*.cpp`, plus `src/main.cpp`
- `orb_slam3_backend.*` only if the fallback is used

- **`SlamBackend` interface**:
  - `init(intrinsics, params)`
  - `track(mono8, stamp) → {state, T_slam_cam (optional)}`
  - `isMetric()`
  - `mapPoints()` returns points with observation counts
  - `keyframes()` returns poses and observed point ids
  - `markers()` returns marker id and pose (empty for ORB-SLAM3)
  - `mapId()`
  - `save(path)`, `load(path)`, `setLocalizationOnly(bool)`
- **Settings**: the node waits for the first `/camera_info` and builds the backend's camera parameters from it, so sim and real share one source of truth. Algorithm params live in `config/<backend>.yaml`: marker dictionary, marker size and feature count for UcoSLAM; ORB settings for the fallback.
- **Image pipeline**: `SensorDataQoS` subscription, convert with `cv_bridge` to mono8, keep only the latest frame in a single slot, and call `track()` from a worker thread. Old frames are dropped.
- **State machine** on `/slam/state`: `WAITING_CAMERA → NOT_INITIALIZED → [SCALE_INIT] → TRACKING ⇄ LOST`. `SCALE_INIT` is skipped when `isMetric()` is true. While `LOST`, keep publishing the **last good** `map → odom`.

## Phase 3: Map frame, scale and `map → odom`

- **Map frame definition**:
  - **UcoSLAM**: the param `origin_marker_id` (e.g. 0) sets `T_map_slam` so that `map` sits on the floor directly below that marker, z up, x along the marker's normal (into the room). The map frame is therefore the same in every session.
  - If `origin_marker_id` is unset, or for ORB-SLAM3: `map ≡ odom` at initialization.
  - Grounding to the floor uses `T_base_cam` from TF: project to planar, with z = 0 and roll and pitch removed.
- **`ScaleEstimator`** (shared code, two roles):
  - UcoSLAM: a **sanity check** only. Compare SLAM displacement with odometry displacement over a sliding window and warn on > 5% disagreement, which usually means a wrong marker size or too few markers.
  - ORB-SLAM3 fallback: the actual scale source.
    - Keep an odometry ring buffer `(stamp, T_odom_cam)`.
    - Rotation is known, `R_odom_slam = R_odom_cam(t_kf0)`, so a straight line is not degenerate.
    - Over keyframe pairs with > 0.5 m of motion, `s = Σ (d_odomᵀ·R·d_slam) / Σ |d_slam|²`, with robust rejection.
    - Refit a global Sim(3) over all keyframes at save time.
- **Pose → TF**:
  - `T_map_base = T_map_slam · S(s)·T_slam_cam · T_cam_base`, projected to SE(2). `s = 1` for UcoSLAM.
  - `T_map_odom = T_map_base · T_odom_base(stamp)⁻¹`.
  - Broadcast at 30 Hz, future-dated by `transform_tolerance`. Also publish `/slam/pose`.

## Phase 4: Mapping mode, occupancy grid

`OccupancyGridBuilder` rebuilds every ~2 s and after loop closure, from `backend.mapPoints()` and `keyframes()` transformed into `map`:
- **Occupied**: points with z in `[0.05, 0.40]` m and at least N observations. With UcoSLAM, marker positions are also marked occupied, since they sit on walls.
- **Free**: 2D log-odds ray-tracing from each keyframe camera position to its observed in-band and floor points. Never trace to ceiling points.
- **Gaps from textureless walls**: cells never seen stay unknown, Nav2 uses `allow_unknown: false`, and the `.pgm` can be hand-edited.
- Publish `/map` (transient-local) while mapping.
- **`/slam/save_map` service**:
  1. ORB-SLAM3 only: refit scale,
  2. `backend.save()` writes `maps/<name>.<ext>`,
  3. write the grid as `.pgm` + `.yaml` in map_server format,
  4. write the sidecar `<name>_slam.yaml` (backend, scale, `T_map_slam`, origin marker, marker size and dictionary, intrinsics hash).

## Phase 5: Navigation mode, localization only

- Load the sidecar and the backend map, then call `setLocalizationOnly(true)`. The map is never modified or overwritten. `map_server` serves the `.pgm`/`.yaml`.
- `map → odom` is published only after relocalization. No `/initialpose` is needed. With UcoSLAM, seeing any known marker relocalizes immediately.

## Phase 6: Nav2 integration (new package `brutusbot_navigation`, config + launch only)

- `config/nav2_params.yaml`:
  - Servers: `bt_navigator`, `planner_server` (Smac 2D or NavFn, `allow_unknown: false`), `controller_server` (Regulated Pure Pursuit), `behavior_server`, `velocity_smoother`, `lifecycle_manager`. **No AMCL.**
  - Footprint 0.30 × 0.25, offset from `base_footprint` per the URDF.
  - Global costmap: static + inflation. Local costmap: rolling window with static + inflation.
  - Conservative speeds: ~0.3 m/s and ≤ 0.6 rad/s, because fast rotation breaks visual tracking. Limit or remove the `spin` behavior.
  - `enable_stamped_cmd_vel: true`, with `cmd_vel` remapped to `/brutusbot_controller/cmd_vel`.
- `launch/navigation.launch.py`: `map_server` (nav mode) + Nav2 servers + lifecycle manager, with a `use_sim_time` arg.
- `brutusbot_slam/launch/slam.launch.py`: args `mode:=mapping|localization`, `map:=<name>`, `use_sim_time`, `viewer`.
- **Bringup**: add `slam:=none|mapping|localization`, `nav:=true|false` and `world:=` to `simulated_robot.launch.py` and `real_robot.launch.py`. Defaults keep today's behavior.
- **Check first**: `brutusbot_controllers.yaml` has `odom_frame_id: /odom`. A leading slash is invalid in TF2. Verify with `ros2 run tf2_ros tf2_echo odom base_footprint` and change it to `odom` if needed.

## Files to create / modify

- `brutusbot_slam/`: `CMakeLists.txt`, `package.xml` (rclcpp, sensor_msgs, nav_msgs, geometry_msgs, std_srvs, tf2_ros, tf2_eigen, cv_bridge), `cmake/Find*.cmake`, `third_party/*.patch`, `src/`, `include/`, `config/{ucoslam.yaml,brutusbot_slam.yaml}`, `launch/slam.launch.py`, `maps/.gitkeep`, `scripts/calibrate_camera_extrinsics.py`
- `brutusbot_navigation/` (new): `config/nav2_params.yaml`, `launch/navigation.launch.py`
- `brutusbot_description/`: `urdf/{camera.xacro,brutusbot.xacro,brutusbot_gazebo.xacro,mobile_base.xacro}`, `config/camera_mount.yaml`, `launch/gazebo.launch.py`, `worlds/{empty,textured_room}.sdf`, `worlds/textured_room_markers.yaml`, `models/{aruco_<id>,calib_checkerboard,room_textures}/`, `scripts/make_marker_models.py`, `CMakeLists.txt`
- `brutusbot_bringup/`: `launch/{simulated_robot,real_robot}.launch.py`, `config/{v4l2_camera.yaml,camera_calibration.yaml}`, `package.xml`
- `brutusbot_controller/config/brutusbot_controllers.yaml`: the `odom_frame_id` fix, only if confirmed
- `CLAUDE.md` / README: new packages, backend build steps, marker printing and layout guide

## Verification

1. **Phase 0**: the go criteria are measured and written down (fps on the Pi, scale error over 3 m, kidnap relocalization, save/load). Backend decision recorded.
2. **Phase 1**: see "Phase 1 done when". In short: `/image_raw` ~30 Hz in sim and on the Pi, the camera TF is correct, markers are detectable in the sim camera, intrinsics < 0.3 px, and the extrinsic calibration is validated in sim and repeatable on the real robot.
3. **Phases 2–3 (sim)**: launch `simulated_robot.launch.py world:=textured_room slam:=mapping` and drive a loop.
   - `/slam/state` reaches `TRACKING`.
   - `/slam/pose` agrees with Gazebo ground truth within ~5 cm and ~2°.
   - The `map` origin lands at the origin marker.
   - The scale sanity check stays below 5%.
4. **Phase 4**: the `/map` grid lines up with the walls. `/slam/save_map` writes the backend map, `.pgm`, `.yaml` and the sidecar.
5. **Phase 5**: restart with `slam:=localization map:=<name>`. The robot relocalizes on the first marker seen, the pose matches ground truth while driving, and the map file is unchanged after shutdown.
6. **Phase 6**: a Nav2 goal from RViz reaches its target in sim. Then repeat on the real robot at low speed.
7. `colcon test --packages-select brutusbot_slam brutusbot_navigation` (lint only), plus a gtest for `ScaleEstimator` and map-frame math with synthetic trajectories (straight line, arc, pure rotation).
