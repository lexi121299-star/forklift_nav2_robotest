#!/usr/bin/env python3
"""Generate an approximate Gazebo world from a 2D occupancy map.

This is intentionally a coarse factory-scene generator, not a CAD importer.
It creates:
  - a large floor matching the map extent,
  - perimeter walls around the map bounds,
  - merged obstacle blocks from occupied pixels.

The generated world is useful for Nav2/Gazebo smoke tests where the simulated
laser should roughly agree with the map used by localization/planning.
"""

import argparse
import math
from collections import defaultdict
from pathlib import Path

import yaml
from PIL import Image


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--output-world", required=True)
    parser.add_argument("--world-name", default="map_factory_approx")
    parser.add_argument("--obstacle-bin-m", type=float, default=2.0)
    parser.add_argument("--min-occupied-pixels-per-bin", type=int, default=5)
    parser.add_argument("--floor-z", type=float, default=-0.02)
    parser.add_argument("--floor-thickness-m", type=float, default=0.04)
    parser.add_argument("--wall-height-m", type=float, default=1.2)
    parser.add_argument("--wall-thickness-m", type=float, default=0.25)
    parser.add_argument("--obstacle-height-m", type=float, default=1.0)
    parser.add_argument("--max-obstacle-blocks", type=int, default=900)
    parser.add_argument(
        "--image-y-origin",
        choices=["top", "bottom"],
        default="top",
        help=(
            "Use top for ROS map-server style image coordinates. Use bottom "
            "when the Gazebo world appears vertically mirrored against RViz."
        ),
    )
    parser.add_argument(
        "--clear-circle",
        action="append",
        default=[],
        metavar="X,Y,R",
        help="Remove generated obstacle blocks within radius R of map/world point X,Y.",
    )
    parser.add_argument(
        "--skip-obstacles",
        action="store_true",
        help="Only generate floor and perimeter walls.",
    )
    return parser.parse_args()


def load_map_info(map_yaml_path):
    with map_yaml_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    image_path = Path(data["image"])
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path
    return {
        "image_path": image_path,
        "resolution": float(data["resolution"]),
        "origin": data["origin"],
        "occupied_thresh": float(data.get("occupied_thresh", 0.65)),
        "negate": int(data.get("negate", 0)),
    }


def pixel_is_occupied(value, negate):
    if negate:
        value = 255 - value
    # map_server trinary occupied threshold: occ = (255 - value) / 255
    return value < 89


def map_xy_from_pixel(px, py, width, height, resolution, origin, image_y_origin):
    # PIL/image y grows downward. Occupancy-grid map y grows upward from origin.
    x = origin[0] + (px + 0.5) * resolution
    if image_y_origin == "top":
        y = origin[1] + (height - py - 0.5) * resolution
    else:
        y = origin[1] + (py + 0.5) * resolution
    return x, y


def parse_clear_circles(clear_circle_args):
    circles = []
    for item in clear_circle_args:
        parts = [part.strip() for part in item.split(",")]
        if len(parts) != 3:
            raise ValueError("--clear-circle expects X,Y,R, got {}".format(item))
        circles.append(tuple(float(part) for part in parts))
    return circles


def is_in_clear_circle(x, y, clear_circles):
    for cx, cy, radius in clear_circles:
        if math.hypot(x - cx, y - cy) <= radius:
            return True
    return False


def collect_obstacle_bins(
    image,
    resolution,
    origin,
    bin_m,
    min_pixels,
    max_blocks,
    negate,
    image_y_origin,
    clear_circles,
):
    width, height = image.size
    bin_px = max(1, int(round(bin_m / resolution)))
    pixels = image.load()
    bins = defaultdict(int)
    for py in range(height):
        for px in range(width):
            if pixel_is_occupied(pixels[px, py], negate):
                bins[(px // bin_px, py // bin_px)] += 1

    kept = [
        (count, bx, by)
        for (bx, by), count in bins.items()
        if count >= min_pixels
    ]
    kept.sort(reverse=True)
    if max_blocks > 0:
        kept = kept[:max_blocks]

    blocks = []
    for count, bx, by in kept:
        center_px = bx * bin_px + (bin_px - 1) * 0.5
        center_py = by * bin_px + (bin_px - 1) * 0.5
        x, y = map_xy_from_pixel(
            center_px,
            center_py,
            width,
            height,
            resolution,
            origin,
            image_y_origin,
        )
        if is_in_clear_circle(x, y, clear_circles):
            continue
        blocks.append({
            "name": "obs_{:04d}_{:04d}".format(bx, by),
            "x": x,
            "y": y,
            "count": count,
        })
    return blocks


def material_xml(name, ambient, diffuse=None):
    diffuse = diffuse or ambient
    return (
        "<material><ambient>{}</ambient><diffuse>{}</diffuse></material>".format(
            ambient,
            diffuse,
        )
    )


def box_link_xml(name, x, y, z, sx, sy, sz, material):
    return (
        '      <link name="{name}"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>'
        '<collision name="collision"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>'
        '<visual name="visual"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>{material}</visual>'
        "</link>\n"
    ).format(
        name=name,
        x=x,
        y=y,
        z=z,
        sx=sx,
        sy=sy,
        sz=sz,
        material=material,
    )


def write_world(path, world_name, width_m, height_m, origin, blocks, args):
    center_x = origin[0] + width_m * 0.5
    center_y = origin[1] + height_m * 0.5
    floor_z = args.floor_z
    floor_thickness = args.floor_thickness_m
    wall_h = args.wall_height_m
    wall_t = args.wall_thickness_m
    obstacle_h = args.obstacle_height_m
    obstacle_size = args.obstacle_bin_m

    floor_mat = material_xml("floor", "0.45 0.46 0.44 1")
    wall_mat = material_xml("wall", "0.32 0.34 0.36 1")
    obstacle_mat = material_xml("obstacle", "0.70 0.58 0.42 1")
    boundary_mat = material_xml("boundary", "0.84 0.74 0.18 1")

    lines = [
        '<?xml version="1.0"?>\n',
        '<sdf version="1.6">\n',
        '  <world name="{}">\n'.format(world_name),
        '    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"><update_rate>10.0</update_rate></plugin>\n',
        '    <scene><ambient>0.50 0.50 0.50 1</ambient><background>0.70 0.78 0.86 1</background><shadows>true</shadows></scene>\n',
        '    <physics name="default_physics" type="ode"><real_time_update_rate>1000</real_time_update_rate><max_step_size>0.001</max_step_size></physics>\n',
        '    <light name="sun" type="directional"><cast_shadows>true</cast_shadows><pose>0 0 20 0 0 0</pose><diffuse>0.82 0.82 0.82 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.45 0.15 -0.88</direction></light>\n',
        '    <model name="map5_floor"><static>true</static>\n',
        box_link_xml(
            "floor",
            center_x,
            center_y,
            floor_z,
            width_m,
            height_m,
            floor_thickness,
            floor_mat,
        ),
        "    </model>\n",
        '    <model name="map5_perimeter_walls"><static>true</static>\n',
        box_link_xml(
            "west_wall",
            origin[0] - wall_t * 0.5,
            center_y,
            wall_h * 0.5,
            wall_t,
            height_m + wall_t * 2.0,
            wall_h,
            wall_mat,
        ),
        box_link_xml(
            "east_wall",
            origin[0] + width_m + wall_t * 0.5,
            center_y,
            wall_h * 0.5,
            wall_t,
            height_m + wall_t * 2.0,
            wall_h,
            wall_mat,
        ),
        box_link_xml(
            "south_wall",
            center_x,
            origin[1] - wall_t * 0.5,
            wall_h * 0.5,
            width_m + wall_t * 2.0,
            wall_t,
            wall_h,
            wall_mat,
        ),
        box_link_xml(
            "north_wall",
            center_x,
            origin[1] + height_m + wall_t * 0.5,
            wall_h * 0.5,
            width_m + wall_t * 2.0,
            wall_t,
            wall_h,
            wall_mat,
        ),
        "    </model>\n",
        '    <model name="map5_obstacle_blocks"><static>true</static>\n',
    ]

    for block in blocks:
        material = obstacle_mat if block["count"] < 50 else boundary_mat
        lines.append(
            box_link_xml(
                block["name"],
                block["x"],
                block["y"],
                obstacle_h * 0.5,
                obstacle_size,
                obstacle_size,
                obstacle_h,
                material,
            )
        )

    lines.extend([
        "    </model>\n",
        "  </world>\n",
        "</sdf>\n",
    ])
    path.write_text("".join(lines), encoding="utf-8")


def main():
    args = parse_args()
    map_yaml_path = Path(args.map_yaml).resolve()
    output_path = Path(args.output_world).resolve()
    info = load_map_info(map_yaml_path)
    image = Image.open(info["image_path"]).convert("L")
    width_px, height_px = image.size
    width_m = width_px * info["resolution"]
    height_m = height_px * info["resolution"]
    clear_circles = parse_clear_circles(args.clear_circle)
    if args.skip_obstacles:
        blocks = []
    else:
        blocks = collect_obstacle_bins(
            image,
            info["resolution"],
            info["origin"],
            args.obstacle_bin_m,
            args.min_occupied_pixels_per_bin,
            args.max_obstacle_blocks,
            info["negate"],
            args.image_y_origin,
            clear_circles,
        )
    write_world(
        output_path,
        args.world_name,
        width_m,
        height_m,
        info["origin"],
        blocks,
        args,
    )
    print(
        "Generated {}: {:.1f}m x {:.1f}m, {} obstacle blocks".format(
            output_path,
            width_m,
            height_m,
            len(blocks),
        )
    )


if __name__ == "__main__":
    main()
