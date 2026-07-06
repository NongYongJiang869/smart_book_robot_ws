#!/usr/bin/env python3
"""
Keepout Mask Publisher

Reads a keepout mask PGM+YAML and publishes:
  1. nav_msgs/OccupancyGrid on /keepout_mask (the mask data)
  2. nav2_msgs/CostmapFilterInfo on /costmap_filter_info (tells KeepoutFilter where to find it)

Both published with transient_local QoS so the KeepoutFilter in the global
costmap receives them even if it starts after this node.

Publishes at 1 Hz and auto-reloads the mask file when it changes on disk.
"""

import os
import yaml
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from std_msgs.msg import Header


KEEPOUT_YAML = '/root/library_keepout_mask.yaml'


def load_keepout_mask(yaml_path):
    """Load keepout mask PGM referenced by YAML. Returns (data, yaml_config)."""
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    pgm_path = config['image']
    img = Image.open(pgm_path)
    data = np.array(img).astype(np.int16)

    return data, config


def pgm_to_occupancy_grid(data, config, stamp):
    """
    Convert PGM pixel array to OccupancyGrid message.

    Mode 'scale' with negate=0:
        occ = (255 - pixel) / 255
        value = int(occ * 100)
    """
    resolution = config['resolution']
    origin = config['origin']  # [x, y, yaw]
    negate = config.get('negate', 0)
    free_thresh = config.get('free_thresh', 0.25)
    occupied_thresh = config.get('occupied_thresh', 0.65)
    mode = config.get('mode', 'trinary')

    height, width = data.shape

    if negate:
        pixel_data = data.astype(np.float64)
    else:
        pixel_data = (255 - data).astype(np.float64)
    occ = pixel_data / 255.0

    if mode == 'trinary':
        grid = np.full((height, width), -1, dtype=np.int8)
        grid[occ <= free_thresh] = 0
        grid[occ >= occupied_thresh] = 100
    else:
        grid = np.clip(occ * 100, 0, 100).astype(np.int8)

    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = 'map'
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = float(origin[0])
    msg.info.origin.position.y = float(origin[1])
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.w = 1.0
    # y-axis flip: PGM row 0 = top, OccupancyGrid row 0 = bottom
    grid = np.flipud(grid)
    msg.data = grid.flatten().tolist()

    return msg


def make_filter_info(stamp):
    """
    Build CostmapFilterInfo message telling KeepoutFilter:
      - type 0 = keepout filter
      - mask is on /keepout_mask topic
      - data conversion: value = occ * 1.0 + 0.0  (identity)
    """
    msg = CostmapFilterInfo()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = 'map'
    msg.type = 0  # KEEPOUT_FILTER
    msg.filter_mask_topic = '/keepout_mask'
    msg.base = 0.0
    msg.multiplier = 1.0
    return msg


def main():
    rclpy.init()

    node = Node('keepout_mask_publisher')

    qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    # Publisher 1: the actual mask OccupancyGrid
    mask_pub = node.create_publisher(OccupancyGrid, '/keepout_mask', qos)

    # Publisher 2: the filter info (tells KeepoutFilter where the mask is)
    info_pub = node.create_publisher(CostmapFilterInfo, '/costmap_filter_info', qos)

    if not os.path.exists(KEEPOUT_YAML):
        node.get_logger().error(f'Keepout mask YAML not found: {KEEPOUT_YAML}')
        raise SystemExit(1)

    data, config = load_keepout_mask(KEEPOUT_YAML)
    node.get_logger().info(
        f'Loaded keepout mask: {config["image"]} '
        f'({data.shape[1]}x{data.shape[0]}, {config["resolution"]}m/px)'
    )

    last_mtime = os.path.getmtime(KEEPOUT_YAML)

    # Use a fixed timestamp to avoid triggering constant filter re-processing
    base_stamp = node.get_clock().now().to_msg()

    def publish():
        nonlocal data, config, last_mtime

        # Reload if file changed
        try:
            mtime = os.path.getmtime(KEEPOUT_YAML)
            if mtime > last_mtime:
                data, config = load_keepout_mask(KEEPOUT_YAML)
                last_mtime = mtime
                node.get_logger().info('Keepout mask reloaded (file changed on disk)')
                # Only re-publish when file actually changed
                info_pub.publish(make_filter_info(base_stamp))
                mask_pub.publish(pgm_to_occupancy_grid(data, config, base_stamp))
                occupied = sum(1 for v in (data <= 89).flat)
                total = data.size
                node.get_logger().info(
                    f'Keepout mask updated: {occupied}/{total} cells '
                    f'({occupied/total*100:.1f}%)'
                )
        except OSError:
            pass

    # Publish once on startup
    info_pub.publish(make_filter_info(base_stamp))
    mask_pub.publish(pgm_to_occupancy_grid(data, config, base_stamp))

    occupied = sum(1 for v in (data <= 89).flat)
    total = data.size
    node.get_logger().info(
        f'Keepout mask publisher running — {occupied} keepout cells '
        f'({occupied/total*100:.1f}%). Publishing on file change only.'
    )

    # Poll file for changes every 2 seconds (don't re-publish unless changed)
    def poll_file():
        nonlocal data, config, last_mtime
        try:
            mtime = os.path.getmtime(KEEPOUT_YAML)
            if mtime > last_mtime:
                data, config = load_keepout_mask(KEEPOUT_YAML)
                last_mtime = mtime
                info_pub.publish(make_filter_info(base_stamp))
                mask_pub.publish(pgm_to_occupancy_grid(data, config, base_stamp))
                occupied = sum(1 for v in (data <= 89).flat)
                node.get_logger().info(
                    f'Keepout mask reloaded: {occupied} cells '
                    f'({occupied/total*100:.1f}%)'
                )
        except OSError:
            pass

    timer = node.create_timer(2.0, poll_file)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
