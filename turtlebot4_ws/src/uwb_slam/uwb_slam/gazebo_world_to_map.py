#!/usr/bin/env python3
"""Rasterize a Gazebo world file into a Nav2-compatible occupancy grid map.

This utility is intentionally lightweight and uses only static collision
geometry from the world/SDF. It is suitable for producing a first-pass 2D
warehouse map that can then be saved as a Nav2 YAML/PGM pair and loaded into
the stack.
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import sys
import warnings
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class Primitive:
    kind: str
    x: float
    y: float
    yaw: float
    width: float = 0.0
    height: float = 0.0
    radius: float = 0.0


def _parse_pose(text: str | None) -> tuple[float, float, float, float, float, float]:
    values = [float(value) for value in (text or '0 0 0 0 0 0').split()]
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])


def _compose_pose(
    parent: tuple[float, float, float, float, float, float],
    child: tuple[float, float, float, float, float, float],
) -> tuple[float, float, float, float, float, float]:
    px, py, pz, proll, ppitch, pyaw = parent
    cx, cy, cz, croll, cpitch, cyaw = child

    cos_yaw = math.cos(pyaw)
    sin_yaw = math.sin(pyaw)
    rx = cos_yaw * cx - sin_yaw * cy
    ry = sin_yaw * cx + cos_yaw * cy

    return (
        px + rx,
        py + ry,
        pz + cz,
        proll + croll,
        ppitch + cpitch,
        pyaw + cyaw,
    )


def _expanded_world_xml(world_path: Path) -> str:
    candidates = [
        ['gz', 'sdf', '-p', str(world_path)],
        ['ign', 'sdf', '-p', str(world_path)],
    ]
    for command in candidates:
        if shutil.which(command[0]) is None:
            continue
        try:
            completed = subprocess.run(command, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError:
            continue
        if completed.stdout.strip():
            return completed.stdout
    return world_path.read_text(encoding='utf-8')


def _iter_collision_primitives(root: ET.Element) -> list[Primitive]:
    primitives: list[Primitive] = []

    for model in root.findall('.//model'):
        model_pose = _parse_pose(model.findtext('pose'))
        for link in model.findall('.//link'):
            link_pose = _compose_pose(model_pose, _parse_pose(link.findtext('pose')))
            for collision in link.findall('collision'):
                collision_pose = _compose_pose(link_pose, _parse_pose(collision.findtext('pose')))
                geometry = collision.find('geometry')
                if geometry is None:
                    continue

                box = geometry.find('box')
                if box is not None:
                    size_text = box.findtext('size', default='0 0 0')
                    sx, sy, _sz = [float(value) for value in size_text.split()[:3]]
                    primitives.append(
                        Primitive('box', collision_pose[0], collision_pose[1], collision_pose[5], width=sx, height=sy)
                    )
                    continue

                cylinder = geometry.find('cylinder')
                if cylinder is not None:
                    radius = float(cylinder.findtext('radius', default='0.0'))
                    primitives.append(
                        Primitive('cylinder', collision_pose[0], collision_pose[1], collision_pose[5], radius=radius)
                    )
                    continue

                sphere = geometry.find('sphere')
                if sphere is not None:
                    radius = float(sphere.findtext('radius', default='0.0'))
                    primitives.append(
                        Primitive('sphere', collision_pose[0], collision_pose[1], collision_pose[5], radius=radius)
                    )
                    continue

                mesh = geometry.find('mesh')
                if mesh is not None:
                    warnings.warn(
                        f'Unsupported mesh collision in {collision.get("name", "<unnamed>")}; skipping rasterization for that element',
                        RuntimeWarning,
                    )

    return primitives


def _world_bounds(primitives: list[Primitive], margin: float) -> tuple[float, float, float, float]:
    if not primitives:
        return -5.0, -5.0, 5.0, 5.0

    xs = []
    ys = []
    for primitive in primitives:
        if primitive.kind == 'box':
            half_w = primitive.width / 2.0
            half_h = primitive.height / 2.0
            corners = [
                (-half_w, -half_h),
                (-half_w, half_h),
                (half_w, -half_h),
                (half_w, half_h),
            ]
            cos_yaw = math.cos(primitive.yaw)
            sin_yaw = math.sin(primitive.yaw)
            for dx, dy in corners:
                xs.append(primitive.x + cos_yaw * dx - sin_yaw * dy)
                ys.append(primitive.y + sin_yaw * dx + cos_yaw * dy)
        else:
            xs.extend([primitive.x - primitive.radius, primitive.x + primitive.radius])
            ys.extend([primitive.y - primitive.radius, primitive.y + primitive.radius])

    return min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin


def _paint_box(grid: np.ndarray, origin_x: float, origin_y: float, resolution: float, primitive: Primitive) -> None:
    half_w = primitive.width / 2.0
    half_h = primitive.height / 2.0
    corners = [
        (-half_w, -half_h),
        (-half_w, half_h),
        (half_w, -half_h),
        (half_w, half_h),
    ]
    cos_yaw = math.cos(primitive.yaw)
    sin_yaw = math.sin(primitive.yaw)
    xs = []
    ys = []
    for dx, dy in corners:
        xs.append(primitive.x + cos_yaw * dx - sin_yaw * dy)
        ys.append(primitive.y + sin_yaw * dx + cos_yaw * dy)

    min_x = int(math.floor((min(xs) - origin_x) / resolution))
    max_x = int(math.ceil((max(xs) - origin_x) / resolution))
    min_y = int(math.floor((min(ys) - origin_y) / resolution))
    max_y = int(math.ceil((max(ys) - origin_y) / resolution))

    min_x = max(min_x, 0)
    min_y = max(min_y, 0)
    max_x = min(max_x, grid.shape[1] - 1)
    max_y = min(max_y, grid.shape[0] - 1)

    if min_x <= max_x and min_y <= max_y:
        grid[min_y : max_y + 1, min_x : max_x + 1] = 0


def _paint_circle(grid: np.ndarray, origin_x: float, origin_y: float, resolution: float, primitive: Primitive) -> None:
    radius_cells = int(math.ceil(primitive.radius / resolution))
    center_x = int(round((primitive.x - origin_x) / resolution))
    center_y = int(round((primitive.y - origin_y) / resolution))

    y_min = max(0, center_y - radius_cells)
    y_max = min(grid.shape[0] - 1, center_y + radius_cells)
    x_min = max(0, center_x - radius_cells)
    x_max = min(grid.shape[1] - 1, center_x + radius_cells)

    radius_sq = radius_cells * radius_cells
    for y in range(y_min, y_max + 1):
        for x in range(x_min, x_max + 1):
            if (x - center_x) ** 2 + (y - center_y) ** 2 <= radius_sq:
                grid[y, x] = 0


def rasterize_world(world_path: Path, resolution: float, margin: float) -> tuple[np.ndarray, tuple[float, float, float]]:
    xml_text = _expanded_world_xml(world_path)
    root = ET.fromstring(xml_text)
    primitives = _iter_collision_primitives(root)
    min_x, min_y, max_x, max_y = _world_bounds(primitives, margin)

    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))
    grid = np.full((height, width), 254, dtype=np.uint8)

    for primitive in primitives:
        if primitive.kind == 'box':
            _paint_box(grid, min_x, min_y, resolution, primitive)
        elif primitive.kind in ('cylinder', 'sphere'):
            _paint_circle(grid, min_x, min_y, resolution, primitive)

    origin = (min_x, min_y, 0.0)
    return grid, origin


def write_map_files(grid: np.ndarray, origin: tuple[float, float, float], output_prefix: Path, resolution: float) -> tuple[Path, Path]:
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    image_path = output_prefix.with_suffix('.pgm')
    yaml_path = output_prefix.with_suffix('.yaml')

    with image_path.open('wb') as image_file:
        image_file.write(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode('ascii'))
        image_file.write(np.flipud(grid).tobytes())

    yaml_text = (
        f'image: {image_path.name}\n'
        'mode: trinary\n'
        f'resolution: {resolution:.6f}\n'
        f'origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
    )
    yaml_path.write_text(yaml_text, encoding='utf-8')
    return image_path, yaml_path


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Rasterize a Gazebo world file into a Nav2 map')
    parser.add_argument('--world', required=True, help='Path to the Gazebo .world or .sdf file')
    parser.add_argument('--output-prefix', required=True, help='Output prefix, e.g. ~/maps/warehouse')
    parser.add_argument('--resolution', type=float, default=0.05, help='Map resolution in meters per cell')
    parser.add_argument('--margin', type=float, default=1.0, help='Padding around the world geometry in meters')
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    # parse_known_args strips ROS-injected --ros-args and remappings that are
    # appended to sys.argv when the node is launched via ros2 launch.
    args, _ = parser.parse_known_args(argv)

    world_path = Path(args.world).expanduser().resolve()
    output_prefix = Path(args.output_prefix).expanduser().resolve()

    if not world_path.exists():
        parser.error(f'World file not found: {world_path}')

    grid, origin = rasterize_world(world_path, args.resolution, args.margin)
    image_path, yaml_path = write_map_files(grid, origin, output_prefix, args.resolution)

    print(f'Wrote {image_path}')
    print(f'Wrote {yaml_path}')
    print('Load the YAML with nav2_map_server or the map_server node.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())