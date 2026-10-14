#!/usr/bin/env python3
"""Generate the SLAM test worlds and their models.

Writes, inside brutusbot_description:
  models/aruco_<id>/            ARUCO_MIP_36h12 marker models (front face at the model origin, normal +x)
  models/room_textures/         procedural, non-repeating wall/floor/box textures
  models/calib_checkerboard/    upright checkerboard for the extrinsic calibration (PLAN.md 1f)
  worlds/textured_room.sdf      5 x 4 m room with boxes and markers
  worlds/textured_room_markers.yaml   ground-truth marker poses (ArUco marker frame in the world)
  worlds/calibration.sdf        empty floor + the checkerboard 1 m in front of the spawn pose
  worlds/calibration_board.yaml board geometry and ground-truth pose

The layout lives in this file, so the SDF and the ground-truth YAML can't disagree.
Textures are seeded, so a rerun gives the same files.

Usage: generate_sim_worlds.py [--markers-zip PATH] [--out PKG_DIR]
(BRUTUSBOT_MARKER_SIZE=<m> overrides the 0.15 m marker size, for experiments.)
"""

import argparse
import math
import os
import shutil
import zipfile
from pathlib import Path

import cv2
import numpy as np
import yaml

MARKER_DICT = "ARUCO_MIP_36h12"
# Black-border edge. The dictionary PNGs are 750 px: a 600 px marker (8 cells of 75 px) and a one-cell white
# quiet zone, the same as the printed sheets.
MARKER_SIZE = float(os.environ.get("BRUTUSBOT_MARKER_SIZE", "0.15"))
MARKER_TILE = MARKER_SIZE * 750.0 / 600.0
MARKER_THICKNESS = 0.002
MARKER_GAP = 0.001  # between the wall surface and the marker back face, avoids z-fighting

ROOM_X = 5.0  # interior size
ROOM_Y = 4.0
WALL_HEIGHT = 1.0
WALL_THICKNESS = 0.1
TEXTURE_PX_PER_M = 400

MARKER_HEIGHTS = (0.17, 0.25)  # alternated; the camera is at 0.21 m
MARKER_SPACING = 0.6
BOX_MARKER_HEIGHT = 0.21

# name, x, y, yaw, size x/y/z, local face (+x/-x/+y/-y) that gets a marker (roughly facing the room centre)
BOXES = [
    ("box_a", 1.2, 1.0, 0.6, (0.40, 0.40, 0.50), "-x"),
    ("box_b", -1.0, -1.0, 0.7, (0.50, 0.30, 0.40), "+x"),
    ("box_c", 1.0, -1.2, -0.87, (0.30, 0.60, 0.35), "-x"),
    ("box_d", -1.4, 1.1, -0.6, (0.45, 0.45, 0.60), "+x"),
]

BOARD_COLS = 9  # inner corners
BOARD_ROWS = 6
BOARD_SQUARE = 0.03
BOARD_MARGIN = 0.03  # white border around the squares
BOARD_THICKNESS = 0.005
BOARD_DISTANCE = 1.0


# ---------------------------------------------------------------------------------------------------------------
# Textures


def clutter_texture(width, height, seed):
    """Posters, shapes, strokes and text on a smoothly varying background: corners at many scales, no repetition."""
    rng = np.random.default_rng(seed)
    low = rng.uniform(90, 200, size=(max(2, height // 160), max(2, width // 160), 3)).astype(np.float32)
    img = cv2.resize(low, (width, height), interpolation=cv2.INTER_CUBIC)
    img = np.clip(img, 0, 255).astype(np.uint8)
    n = int(width * height / 2500)
    for _ in range(n):
        color = tuple(int(c) for c in rng.integers(0, 256, 3))
        kind = rng.choice(["rect", "frame", "circle", "line", "text"], p=[0.3, 0.15, 0.2, 0.2, 0.15])
        x, y = int(rng.integers(0, width)), int(rng.integers(0, height))
        s = int(rng.integers(6, 120))
        if kind == "rect":
            cv2.rectangle(img, (x, y), (x + s, y + int(s * rng.uniform(0.3, 2.0))), color, -1)
        elif kind == "frame":
            cv2.rectangle(img, (x, y), (x + s, y + int(s * rng.uniform(0.5, 1.5))), color, int(rng.integers(2, 8)))
        elif kind == "circle":
            cv2.circle(img, (x, y), s // 2, color, -1 if rng.random() < 0.6 else int(rng.integers(2, 6)))
        elif kind == "line":
            a = rng.uniform(0, math.pi)
            p2 = (int(x + 2 * s * math.cos(a)), int(y + 2 * s * math.sin(a)))
            cv2.line(img, (x, y), p2, color, int(rng.integers(2, 7)))
        else:
            text = "".join(rng.choice(list("ABCDEFGHKMNPRSTUVWXYZ0123456789"), int(rng.integers(2, 7))))
            cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, s / 60.0, color, max(1, s // 25))
    noise = rng.normal(0, 5, img.shape)
    return np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)


def floor_texture(width, height, seed):
    """Concrete-like floor: low-contrast stains and speckles, no tile grid."""
    rng = np.random.default_rng(seed)
    low = rng.uniform(110, 170, size=(max(2, height // 100), max(2, width // 100), 1)).astype(np.float32)
    img = cv2.resize(low, (width, height), interpolation=cv2.INTER_CUBIC)
    img = np.repeat(img[:, :, None] if img.ndim == 2 else img, 3, axis=2) * np.array([0.95, 1.0, 1.05], np.float32)
    img = np.clip(img, 0, 255).astype(np.uint8)
    for _ in range(int(width * height / 400)):
        c = int(rng.integers(40, 230))
        color = (c, c + int(rng.integers(-15, 15)), c + int(rng.integers(-15, 15)))
        color = tuple(int(np.clip(v, 0, 255)) for v in color)
        axes = (int(rng.integers(2, 18)), int(rng.integers(2, 18)))
        cv2.ellipse(img, (int(rng.integers(0, width)), int(rng.integers(0, height))), axes,
                    float(rng.uniform(0, 180)), 0, 360, color, -1)
    noise = rng.normal(0, 6, img.shape)
    return np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)


def checkerboard_texture(px_per_square=50):
    sq_x, sq_y = BOARD_COLS + 1, BOARD_ROWS + 1
    m = int(round(BOARD_MARGIN / BOARD_SQUARE * px_per_square))
    img = np.full((sq_y * px_per_square + 2 * m, sq_x * px_per_square + 2 * m), 255, np.uint8)
    for r in range(sq_y):
        for c in range(sq_x):
            if (r + c) % 2 == 0:
                y0, x0 = m + r * px_per_square, m + c * px_per_square
                img[y0:y0 + px_per_square, x0:x0 + px_per_square] = 0
    return img


# ---------------------------------------------------------------------------------------------------------------
# Geometry


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ArUco marker frame (x right, y up, z out of the marker) in the marker model frame (x = normal, z = up)
R_MODEL_ARUCO = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def quat_xyzw(r):
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return [round(float(v), 6) for v in (x, y, z, w)]


def wall_markers():
    """(x, y, yaw) of the marker front face, going counter-clockwise from the front wall centre (id 0)."""
    hx, hy = ROOM_X / 2.0, ROOM_Y / 2.0
    off = MARKER_GAP + MARKER_THICKNESS
    n_y = int(round((hy - 0.2) / MARKER_SPACING))
    ys = [i * MARKER_SPACING for i in range(-n_y, n_y + 1)]  # front/back walls, includes 0
    n_x = int(math.floor((ROOM_X - 0.4) / MARKER_SPACING)) + 1
    x0 = (n_x - 1) * MARKER_SPACING / 2.0
    xs = [x0 - i * MARKER_SPACING for i in range(n_x)]  # side walls, +x to -x
    out = [(hx - off, y, math.pi) for y in ys if y >= 0]  # front wall, centre to left corner
    out += [(x, hy - off, -math.pi / 2.0) for x in xs]  # left wall
    out += [(-hx + off, y, 0.0) for y in reversed(ys)]  # back wall
    out += [(x, -hy + off, math.pi / 2.0) for x in reversed(xs)]  # right wall
    out += [(hx - off, y, math.pi) for y in ys if y < 0]  # front wall, right corner to centre
    return out


def box_marker(box):
    _, bx, by, yaw, (sx, sy, _), face = box
    sign = 1.0 if face[0] == "+" else -1.0
    axis = 0 if face[1] == "x" else 1
    half = (sx, sy)[axis] / 2.0 + MARKER_GAP + MARKER_THICKNESS
    local = np.zeros(3)
    local[axis] = sign * half
    p = rot_z(yaw) @ local
    face_yaw = yaw + (0.0 if axis == 0 else math.pi / 2.0) + (0.0 if sign > 0 else math.pi)
    return bx + p[0], by + p[1], math.atan2(math.sin(face_yaw), math.cos(face_yaw))


# ---------------------------------------------------------------------------------------------------------------
# SDF snippets


def f(v):
    return f"{v:.6g}"


def pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    return f"<pose>{f(x)} {f(y)} {f(z)} {f(roll)} {f(pitch)} {f(yaw)}</pose>"


def textured_material(uri):
    return f"""<material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
            <pbr><metal><albedo_map>{uri}</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr>
          </material>"""


def box_link(name, xyz_rpy, size, uri, collision=True):
    sx, sy, sz = size
    geom = f"<geometry><box><size>{f(sx)} {f(sy)} {f(sz)}</size></box></geometry>"
    col = f"""
        <collision name="collision">
          {geom}
        </collision>""" if collision else ""
    return f"""
      <link name="{name}">
        {pose(*xyz_rpy)}{col}
        <visual name="visual">
          {geom}
          {textured_material(uri)}
        </visual>
      </link>"""


WORLD_HEADER = """<?xml version="1.0" ?>
<!-- Generated by brutusbot_description/scripts/generate_sim_worlds.py; edit the script, not this file. -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.7 0.7 0.7 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.25 -0.88</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.4 0.4 0.4 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.45 -0.35 -0.82</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""

WORLD_FOOTER = """
  </world>
</sdf>
"""


def include(uri, name, p):
    return f"""
    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      {p}
    </include>"""


def model_config(name, description):
    return f"""<?xml version="1.0"?>
<!-- Generated by brutusbot_description/scripts/generate_sim_worlds.py -->
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{description}</description>
</model>
"""


# ---------------------------------------------------------------------------------------------------------------
# Writers


def write_marker_model(models_dir, marker_id, png_bytes):
    name = f"aruco_{marker_id}"
    d = models_dir / name
    (d / "materials" / "textures").mkdir(parents=True, exist_ok=True)
    (d / "materials" / "textures" / f"{name}.png").write_bytes(png_bytes)
    (d / "model.config").write_text(model_config(name, f"{MARKER_DICT} marker {marker_id}, {MARKER_SIZE} m black edge"))
    uri = f"model://{name}/materials/textures/{name}.png"
    (d / "model.sdf").write_text(f"""<?xml version="1.0"?>
<!-- Generated by brutusbot_description/scripts/generate_sim_worlds.py -->
<!-- Origin: centre of the printed (front) face. +x: face normal. {MARKER_SIZE} m black edge, one-cell white quiet zone. -->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        {pose(-MARKER_THICKNESS / 2.0, 0, 0)}
        <geometry><box><size>{f(MARKER_THICKNESS)} {f(MARKER_TILE)} {f(MARKER_TILE)}</size></box></geometry>
        {textured_material(uri)}
      </visual>
    </link>
  </model>
</sdf>
""")


def write_textured_room(pkg, marker_pngs):
    models_dir = pkg / "models"
    tex_dir = models_dir / "room_textures" / "materials" / "textures"
    tex_dir.mkdir(parents=True, exist_ok=True)
    (models_dir / "room_textures" / "model.config").write_text(
        model_config("room_textures", "Procedural textures for textured_room.sdf (not a spawnable model)"))

    def tex_uri(name):
        return f"model://room_textures/materials/textures/{name}"

    hx, hy = ROOM_X / 2.0, ROOM_Y / 2.0
    ppm = TEXTURE_PX_PER_M
    walls = [  # name, centre, yaw, length along the wall
        ("wall_front", (hx + WALL_THICKNESS / 2.0, 0.0), math.pi / 2.0, ROOM_Y + 2 * WALL_THICKNESS),
        ("wall_left", (0.0, hy + WALL_THICKNESS / 2.0), 0.0, ROOM_X),
        ("wall_back", (-hx - WALL_THICKNESS / 2.0, 0.0), math.pi / 2.0, ROOM_Y + 2 * WALL_THICKNESS),
        ("wall_right", (0.0, -hy - WALL_THICKNESS / 2.0), 0.0, ROOM_X),
    ]
    links = []
    for i, (name, (cx, cy), yaw, length) in enumerate(walls):
        cv2.imwrite(str(tex_dir / f"{name}.jpg"), clutter_texture(int(length * ppm), int(WALL_HEIGHT * ppm), 100 + i),
                    [cv2.IMWRITE_JPEG_QUALITY, 88])
        links.append(box_link(name, (cx, cy, WALL_HEIGHT / 2.0, 0, 0, yaw), (length, WALL_THICKNESS, WALL_HEIGHT),
                              tex_uri(f"{name}.jpg")))
    cv2.imwrite(str(tex_dir / "floor.jpg"), floor_texture(int(ROOM_X * ppm), int(ROOM_Y * ppm), 200),
                [cv2.IMWRITE_JPEG_QUALITY, 88])
    links.append(box_link("floor", (0, 0, -0.0005, 0, 0, 0), (ROOM_X, ROOM_Y, 0.002), tex_uri("floor.jpg"),
                          collision=False))
    for i, (name, bx, by, yaw, size, _) in enumerate(BOXES):
        cv2.imwrite(str(tex_dir / f"{name}.jpg"), clutter_texture(512, 512, 300 + i), [cv2.IMWRITE_JPEG_QUALITY, 88])
        links.append(box_link(name, (bx, by, size[2] / 2.0, 0, 0, yaw), size, tex_uri(f"{name}.jpg")))

    placements = []  # (id, x, y, z, yaw, mounted on)
    for i, (x, y, yaw) in enumerate(wall_markers()):
        placements.append((i, x, y, MARKER_HEIGHTS[i % 2], yaw, "wall"))
    for box in BOXES:
        x, y, yaw = box_marker(box)
        placements.append((len(placements), x, y, BOX_MARKER_HEIGHT, yaw, box[0]))

    sdf = WORLD_HEADER.format(name="textured_room")
    sdf += f"""
    <model name="room">
      <static>true</static>{''.join(links)}
    </model>
"""
    gt = []
    for marker_id, x, y, z, yaw, on in placements:
        write_marker_model(models_dir, marker_id, marker_pngs[marker_id])
        sdf += include(f"model://aruco_{marker_id}", f"aruco_{marker_id}", pose(x, y, z, 0, 0, yaw))
        r = rot_z(yaw) @ R_MODEL_ARUCO
        gt.append({"id": marker_id, "mount": on, "position": [round(float(v), 6) for v in (x, y, z)],
                   "orientation_xyzw": quat_xyzw(r), "yaw": round(float(yaw), 6)})
    sdf += WORLD_FOOTER
    (pkg / "worlds" / "textured_room.sdf").write_text(sdf)

    header = (
        "# Generated by brutusbot_description/scripts/generate_sim_worlds.py; edit the script, not this file.\n"
        "# Ground-truth marker poses in textured_room.sdf (world frame = Gazebo world; the robot spawns at the\n"
        "# origin facing +x). position/orientation: the ArUco marker frame (origin at the marker centre, x right,\n"
        "# y up, z out of the marker, as seen by a viewer facing it). yaw: heading of the marker normal.\n")
    data = {"dictionary": MARKER_DICT, "marker_size": MARKER_SIZE, "markers": gt}
    (pkg / "worlds" / "textured_room_markers.yaml").write_text(header + yaml.safe_dump(data, sort_keys=False, default_flow_style=None))
    return len(placements)


def write_calibration(pkg):
    models_dir = pkg / "models"
    d = models_dir / "calib_checkerboard"
    (d / "materials" / "textures").mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(d / "materials" / "textures" / "checkerboard.png"), checkerboard_texture())
    width = (BOARD_COLS + 1) * BOARD_SQUARE + 2 * BOARD_MARGIN
    height = (BOARD_ROWS + 1) * BOARD_SQUARE + 2 * BOARD_MARGIN
    bottom_row = BOARD_MARGIN + BOARD_SQUARE
    (d / "model.config").write_text(model_config(
        "calib_checkerboard", f"{BOARD_COLS}x{BOARD_ROWS} inner corners, {BOARD_SQUARE} m squares, standing upright"))
    (d / "model.sdf").write_text(f"""<?xml version="1.0"?>
<!-- Generated by brutusbot_description/scripts/generate_sim_worlds.py -->
<!-- Origin: bottom edge centre of the printed (front) face, on the floor. +x: face normal. -->
<sdf version="1.9">
  <model name="calib_checkerboard">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        {pose(-BOARD_THICKNESS / 2.0, 0, height / 2.0)}
        <geometry><box><size>{f(BOARD_THICKNESS)} {f(width)} {f(height)}</size></box></geometry>
        {textured_material("model://calib_checkerboard/materials/textures/checkerboard.png")}
      </visual>
    </link>
  </model>
</sdf>
""")
    sdf = WORLD_HEADER.format(name="calibration")
    sdf += include("model://calib_checkerboard", "calib_checkerboard", pose(BOARD_DISTANCE, 0, 0, 0, 0, math.pi))
    sdf += WORLD_FOOTER
    (pkg / "worlds" / "calibration.sdf").write_text(sdf)
    header = ("# Generated by brutusbot_description/scripts/generate_sim_worlds.py; edit the script, not this file.\n"
              "# Checkerboard in calibration.sdf. pose: bottom edge centre of the front face, yaw of its normal.\n")
    data = {"cols": BOARD_COLS, "rows": BOARD_ROWS, "square": BOARD_SQUARE, "margin": BOARD_MARGIN,
            "bottom_corner_row_height": round(bottom_row, 6), "width": round(width, 6), "height": round(height, 6),
            "pose": {"x": BOARD_DISTANCE, "y": 0.0, "z": 0.0, "yaw": round(math.pi, 6)}}
    (pkg / "worlds" / "calibration_board.yaml").write_text(header + yaml.safe_dump(data, sort_keys=False, default_flow_style=None))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--markers-zip", default=str(Path.home() / "dev/ucoslam_data/markers/aruco_mip_36h12_dict.zip"))
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1]))
    args = ap.parse_args()

    pkg = Path(args.out)
    models_dir = pkg / "models"
    for old in list(models_dir.glob("aruco_*")) + [models_dir / "room_textures", models_dir / "calib_checkerboard"]:
        shutil.rmtree(old, ignore_errors=True)
    (pkg / "worlds").mkdir(exist_ok=True)

    marker_pngs = {}
    with zipfile.ZipFile(args.markers_zip) as z:
        for info in z.infolist():
            stem = Path(info.filename).stem
            if info.filename.endswith(".png") and stem.startswith("aruco_mip_36h12_"):
                marker_pngs[int(stem.rsplit("_", 1)[1])] = z.read(info)

    n = write_textured_room(pkg, marker_pngs)
    write_calibration(pkg)
    print(f"wrote {n} markers, textured_room and calibration worlds under {pkg}")


if __name__ == "__main__":
    main()
