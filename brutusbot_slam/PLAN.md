# Plan: Monocular visual SLAM for Brutusbot (UcoSLAM-first, ORB-SLAM3 fallback), integrated with Nav2

This is a living document. Run it one phase at a time. When a phase is done, update the status table. Record any change of direction in the decision log and edit the affected phase sections.

## Status

| Phase | Description | Status | Notes |
|---|---|---|---|
| 0 | Backend spike (UcoSLAM go/no-go) | Done | **Go for UcoSLAM** (2026-10-11): all 6 criteria pass; 0b fallback not needed |
| 1 | Camera + markers (sim and real) | Not started | |
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

- **URDF** `brutusbot_description/urdf/camera.xacro`, included from `brutusbot.xacro`:
  - `camera_link` at the front of `base_link`, looking forward and pitched ~10° down,
  - `camera_optical_link` with rpy `-π/2 0 -π/2`,
  - the mount pose in xacro properties, because TF supplies `T_base_cam`.
- **Sim**: in `brutusbot_gazebo.xacro`, add a gz `camera` sensor (640×480, 30 Hz, hfov ~1.3 rad). Bridge it to `/image_raw` and `/camera_info` with `ros_gz_bridge` in `gazebo.launch.py`.
- **World** `brutusbot_description/worlds/textured_room.sdf`: textured walls and floor, a few boxes, and **ArUco marker models** (thin boxes with marker PNG textures at a known size, mounted at camera height). Add a `world` launch arg to `gazebo.launch.py`; the default stays `empty.sdf`.
- **Real**: `brutusbot_bringup/config/v4l2_camera.yaml` sets `image_size [640,480]`, fixed exposure if possible, `camera_frame_id: camera_optical_link`, and `camera_info_url` pointing to `config/camera_calibration.yaml`. Add the node to `real_robot.launch.py`.
- **Calibration**: `sudo apt install ros-jazzy-camera-calibration`, then `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=/image_raw`.
- **Marker layout guide** (README): at least one marker visible from the start pose, then one every ~2–3 m along driven routes and at goal areas, mounted at roughly camera height. Avoid steep viewing angles.

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

- `brutusbot_slam/`: `CMakeLists.txt`, `package.xml` (rclcpp, sensor_msgs, nav_msgs, geometry_msgs, std_srvs, tf2_ros, tf2_eigen, cv_bridge), `cmake/Find*.cmake`, `third_party/*.patch`, `src/`, `include/`, `config/{ucoslam.yaml,brutusbot_slam.yaml}`, `launch/slam.launch.py`, `maps/.gitkeep`
- `brutusbot_navigation/` (new): `config/nav2_params.yaml`, `launch/navigation.launch.py`
- `brutusbot_description/`: `urdf/{camera.xacro,brutusbot.xacro,brutusbot_gazebo.xacro}`, `launch/gazebo.launch.py`, `worlds/textured_room.sdf` + marker models/textures
- `brutusbot_bringup/`: `launch/{simulated_robot,real_robot}.launch.py`, `config/{v4l2_camera.yaml,camera_calibration.yaml}`, `package.xml`
- `brutusbot_controller/config/brutusbot_controllers.yaml`: the `odom_frame_id` fix, only if confirmed
- `CLAUDE.md` / README: new packages, backend build steps, marker printing and layout guide

## Verification

1. **Phase 0**: the go criteria are measured and written down (fps on the Pi, scale error over 3 m, kidnap relocalization, save/load). Backend decision recorded.
2. **Phase 1**: `/image_raw` runs at ~30 Hz in sim, the camera TF is correct in RViz, markers are visible in the sim camera, and the real camera is calibrated.
3. **Phases 2–3 (sim)**: launch `simulated_robot.launch.py world:=textured_room slam:=mapping` and drive a loop.
   - `/slam/state` reaches `TRACKING`.
   - `/slam/pose` agrees with Gazebo ground truth within ~5 cm and ~2°.
   - The `map` origin lands at the origin marker.
   - The scale sanity check stays below 5%.
4. **Phase 4**: the `/map` grid lines up with the walls. `/slam/save_map` writes the backend map, `.pgm`, `.yaml` and the sidecar.
5. **Phase 5**: restart with `slam:=localization map:=<name>`. The robot relocalizes on the first marker seen, the pose matches ground truth while driving, and the map file is unchanged after shutdown.
6. **Phase 6**: a Nav2 goal from RViz reaches its target in sim. Then repeat on the real robot at low speed.
7. `colcon test --packages-select brutusbot_slam brutusbot_navigation` (lint only), plus a gtest for `ScaleEstimator` and map-frame math with synthetic trajectories (straight line, arc, pure rotation).
