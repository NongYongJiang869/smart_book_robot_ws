#!/usr/bin/env python3
"""
手动编辑 SLAM 地图 或 禁区蒙版 (keepout mask)。

== 禁区蒙版 (默认) ==
禁区蒙版是一个与 SLAM 地图同尺寸的独立 PGM 文件，只影响路径规划，
不修改原始地图 → AMCL 定位不受影响。
修改后导航自动生效（keepout_mask_publisher 每秒重新加载文件）。

用法:
    # 查看地图信息
    python3 edit_map.py --info

    # 查看禁区蒙版信息
    python3 edit_map.py --show-keepout

    # 添加矩形禁区（世界坐标，米）
    python3 edit_map.py --add-keepout-rect x1 y1 x2 y2

    # 添加圆形禁区
    python3 edit_map.py --add-keepout-circle cx cy radius

    # 清除指定区域的禁区
    python3 edit_map.py --clear-keepout-rect x1 y1 x2 y2

    # 预览修改（不保存）
    python3 edit_map.py --add-keepout-rect 1.0 -2.0 3.0 -1.5 --dry-run

    # 侵蚀禁区自由空间边缘 → 填缝
    python3 edit_map.py --erode-keepout-free 3

== 实际地图编辑（谨慎使用！会影响定位）==
    # 添加障碍物到实际地图（加 --edit-map 标记）
    python3 edit_map.py --edit-map --add-rect x1 y1 x2 y2

    # 填充地图灰色区域为障碍物
    python3 edit_map.py --edit-map --fill-unknown x1 y1 x2 y2
"""

import argparse
import shutil
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import binary_erosion, binary_dilation

MAP_YAML = "/root/library_map.yaml"
MAP_PGM = "/root/library_map.pgm"
KEEPOUT_PGM = "/root/library_keepout_mask.pgm"

RESOLUTION = 0.05
ORIGIN_X, ORIGIN_Y = -20, -20


def world_to_pixel(wx, wy):
    """世界坐标 → 像素坐标"""
    px = int((wx - ORIGIN_X) / RESOLUTION)
    py = int((wy - ORIGIN_Y) / RESOLUTION)
    return px, py


def pixel_to_world(px, py):
    """像素坐标 → 世界坐标"""
    wx = ORIGIN_X + px * RESOLUTION
    wy = ORIGIN_Y + py * RESOLUTION
    return wx, wy


def load_pgm(path):
    img = Image.open(path)
    return img, np.array(img).copy()


def save_pgm(data, path):
    img = Image.fromarray(data)
    img.save(path)
    print(f"✅ 已保存: {path}")


def backup_pgm(path):
    backup = path.replace('.pgm', f'.backup-{datetime.now():%m%d-%H%M}.pgm')
    shutil.copy(path, backup)
    print(f"📦 已备份: {backup}")


# ════════════════════════════════════════════════════════════════
#  Keepout mask 操作（默认，不修改实际地图）
# ════════════════════════════════════════════════════════════════

def ensure_keepout_mask(ref_shape):
    """确保禁区蒙版存在，不存在则创建空白的。"""
    if Path(KEEPOUT_PGM).exists():
        data = load_pgm(KEEPOUT_PGM)[1]
        if data.shape == ref_shape:
            return data
        print(f"⚠️  禁区蒙版尺寸 {data.shape} 与地图 {ref_shape} 不匹配，重新创建")
    data = np.full(ref_shape, 254, dtype=np.uint8)
    Image.fromarray(data).save(KEEPOUT_PGM)
    print(f"📝 创建空白禁区蒙版: {KEEPOUT_PGM}")
    return data


def add_keepout_rect(keepout, x1, y1, x2, y2):
    """在禁区蒙版上添加矩形禁区（涂黑 = pixel=0）"""
    px1, py1 = world_to_pixel(x1, y1)
    px2, py2 = world_to_pixel(x2, y2)
    x_min, x_max = sorted([px1, px2])
    y_min, y_max = sorted([py1, py2])
    x_min = max(0, x_min); x_max = min(keepout.shape[1] - 1, x_max)
    y_min = max(0, y_min); y_max = min(keepout.shape[0] - 1, y_max)
    count = (x_max - x_min + 1) * (y_max - y_min + 1)
    keepout[y_min:y_max+1, x_min:x_max+1] = 0
    print(f"  🚫 禁区矩形 ({x1:.2f},{y1:.2f})→({x2:.2f},{y2:.2f}) "
          f"→ 像素 [{x_min},{y_min}]→[{x_max},{y_max}]，覆盖 {count} px")
    return keepout


def add_keepout_circle(keepout, cx, cy, radius):
    """在禁区蒙版上添加圆形禁区"""
    pcx, pcy = world_to_pixel(cx, cy)
    p_radius = int(radius / RESOLUTION)
    y_idx, x_idx = np.ogrid[:keepout.shape[0], :keepout.shape[1]]
    dist = np.sqrt((x_idx - pcx)**2 + (y_idx - pcy)**2)
    mask = dist <= p_radius
    count = mask.sum()
    keepout[mask] = 0
    print(f"  🚫 禁区圆形 中心({cx:.2f},{cy:.2f}) R={radius:.2f}m "
          f"→ 像素R={p_radius}px，覆盖 {count} px")
    return keepout


def clear_keepout_rect(keepout, x1, y1, x2, y2):
    """清除指定区域的禁区（恢复白色 = pixel=254）"""
    px1, py1 = world_to_pixel(x1, y1)
    px2, py2 = world_to_pixel(x2, y2)
    x_min, x_max = sorted([px1, px2])
    y_min, y_max = sorted([py1, py2])
    x_min = max(0, x_min); x_max = min(keepout.shape[1] - 1, x_max)
    y_min = max(0, y_min); y_max = min(keepout.shape[0] - 1, y_max)
    keepout[y_min:y_max+1, x_min:x_max+1] = 254
    print(f"  ✅ 清除禁区 ({x1:.2f},{y1:.2f})→({x2:.2f},{y2:.2f})")
    return keepout


def erode_keepout_free(keepout, margin_cells):
    """禁区蒙版自由空间侵蚀（填窄缝）——增加禁区覆盖"""
    free_mask = keepout >= 229
    eroded = binary_erosion(free_mask, iterations=margin_cells)
    filled = ~eroded & free_mask
    keepout[filled] = 0
    print(f"  🚫 禁区侵蚀 {margin_cells} 格 ({margin_cells*RESOLUTION:.2f}m)，"
          f"填了 {filled.sum()} px")
    return keepout


# ════════════════════════════════════════════════════════════════
#  实际地图操作（仅 --edit-map 时启用）
# ════════════════════════════════════════════════════════════════

def add_obstacle_rect(data, x1, y1, x2, y2, only_unknown=False):
    """添加矩形障碍物到实际地图（涂黑 = pixel=0 → OCCUPIED）"""
    px1, py1 = world_to_pixel(x1, y1)
    px2, py2 = world_to_pixel(x2, y2)
    x_min, x_max = sorted([px1, px2])
    y_min, y_max = sorted([py1, py2])
    x_min = max(0, x_min); x_max = min(data.shape[1] - 1, x_max)
    y_min = max(0, y_min); y_max = min(data.shape[0] - 1, y_max)

    if only_unknown:
        region = data[y_min:y_max+1, x_min:x_max+1]
        mask = region == 205
        region[mask] = 0
        data[y_min:y_max+1, x_min:x_max+1] = region
        count = mask.sum()
    else:
        count = (x_max - x_min + 1) * (y_max - y_min + 1)
        data[y_min:y_max+1, x_min:x_max+1] = 0

    print(f"  ⬛ 障碍物 ({x1:.2f},{y1:.2f})→({x2:.2f},{y2:.2f}) "
          f"→ 像素 [{x_min},{y_min}]→[{x_max},{y_max}]，覆盖 {count} px")
    return data


def add_obstacle_circle(data, cx, cy, radius):
    """添加圆形障碍物到实际地图"""
    pcx, pcy = world_to_pixel(cx, cy)
    p_radius = int(radius / RESOLUTION)
    y_idx, x_idx = np.ogrid[:data.shape[0], :data.shape[1]]
    dist = np.sqrt((x_idx - pcx)**2 + (y_idx - pcy)**2)
    mask = dist <= p_radius
    count = mask.sum()
    data[mask] = 0
    print(f"  ⬛ 圆形 中心({cx:.2f},{cy:.2f}) R={radius:.2f}m "
          f"→ 像素R={p_radius}px，覆盖 {count} px")
    return data


def erode_free_space(data, margin_cells):
    """将实际地图自由空间边缘收缩，填上窄缝"""
    free_mask = data >= 229
    eroded = binary_erosion(free_mask, iterations=margin_cells)
    filled = ~eroded & free_mask
    data[filled] = 0
    print(f"  ⬛ 侵蚀 {margin_cells} 格 ({margin_cells*RESOLUTION:.2f}m)，"
          f"填了 {filled.sum()} px")
    return data


def fill_unknown_as_obstacle(data, x1, y1, x2, y2):
    """将实际地图指定区域内的灰色变成黑色"""
    return add_obstacle_rect(data, x1, y1, x2, y2, only_unknown=True)


# ════════════════════════════════════════════════════════════════
#  显示
# ════════════════════════════════════════════════════════════════

def show_info(data, label="地图"):
    """显示数据统计"""
    free = (data >= 229).sum()
    unknown = ((data >= 90) & (data <= 228)).sum()
    occupied = (data <= 89).sum()
    total = data.size
    print(f"{label}: {data.shape[1]}×{data.shape[0]} px, {RESOLUTION}m/px")
    print(f"覆盖: {ORIGIN_X}→{ORIGIN_X+data.shape[1]*RESOLUTION:.0f}m, "
          f"{ORIGIN_Y}→{ORIGIN_Y+data.shape[0]*RESOLUTION:.0f}m")
    print(f"🟢 空闲:   {free:>6} ({free/total*100:5.1f}%)")
    print(f"⚪ 未知:   {unknown:>6} ({unknown/total*100:5.1f}%)")
    print(f"🔴 障碍物: {occupied:>6} ({occupied/total*100:5.1f}%)")


# ════════════════════════════════════════════════════════════════
#  main
# ════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="手动编辑 SLAM 地图 / 禁区蒙版",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 edit_map.py --add-keepout-rect 1 -2 3 -1.5    # 添加矩形禁区
  python3 edit_map.py --clear-keepout-rect 1 -2 3 -1.5  # 清除禁区
  python3 edit_map.py --show-keepout                     # 查看禁区信息
  python3 edit_map.py --edit-map --add-rect 1 -2 3 -1.5 # ⚠️ 修改实际地图
        """,
    )
    parser.add_argument("--info", action="store_true", help="显示实际地图信息")

    # 禁区蒙版操作（默认）
    parser.add_argument("--show-keepout", action="store_true", help="显示禁区蒙版信息")
    parser.add_argument("--add-keepout-rect", nargs=4, type=float,
                        metavar=("X1","Y1","X2","Y2"), action="append",
                        help="添加矩形禁区")
    parser.add_argument("--add-keepout-circle", nargs=3, type=float,
                        metavar=("CX","CY","R"), action="append",
                        help="添加圆形禁区")
    parser.add_argument("--clear-keepout-rect", nargs=4, type=float,
                        metavar=("X1","Y1","X2","Y2"), action="append",
                        help="清除指定区域禁区")
    parser.add_argument("--erode-keepout-free", type=int, metavar="CELLS",
                        help="禁区蒙版自由空间侵蚀（填缝）")

    # 实际地图操作（需 --edit-map）
    parser.add_argument("--edit-map", action="store_true",
                        help="⚠️ 修改实际地图（影响定位！默认操作禁区蒙版）")
    parser.add_argument("--add-rect", nargs=4, type=float,
                        metavar=("X1","Y1","X2","Y2"), action="append",
                        help="添加矩形障碍物到实际地图")
    parser.add_argument("--add-circle", nargs=3, type=float,
                        metavar=("CX","CY","R"), action="append",
                        help="添加圆形障碍物到实际地图")
    parser.add_argument("--fill-unknown", nargs=4, type=float,
                        metavar=("X1","Y1","X2","Y2"), action="append",
                        help="将矩形内灰色改为黑色（实际地图）")
    parser.add_argument("--erode-free", type=int, metavar="CELLS",
                        help="侵蚀实际地图自由空间边缘")

    parser.add_argument("--dry-run", action="store_true", help="预览不保存")
    parser.add_argument("--backup", action="store_true", help="修改前备份")

    args = parser.parse_args()

    # ── 纯查询操作 ──
    if args.info and not any([
        args.add_rect, args.add_circle, args.fill_unknown, args.erode_free,
        args.add_keepout_rect, args.add_keepout_circle,
        args.clear_keepout_rect, args.erode_keepout_free,
    ]):
        _, data = load_pgm(MAP_PGM)
        show_info(data, "实际地图")
        return

    if args.show_keepout:
        _, data = load_pgm(KEEPOUT_PGM)
        show_info(data, "禁区蒙版")
        return

    # ── 判断操作目标 ──
    has_map_ops = any([args.add_rect, args.add_circle,
                       args.fill_unknown, args.erode_free])
    has_keepout_ops = any([args.add_keepout_rect, args.add_keepout_circle,
                           args.clear_keepout_rect, args.erode_keepout_free])

    if not has_map_ops and not has_keepout_ops:
        parser.print_help()
        return

    if has_map_ops and not args.edit_map:
        print("⚠️  你在尝试修改实际地图！这会直接影响 AMCL 定位。")
        print("   如果要修改禁区蒙版，请用 --add-keepout-rect / --clear-keepout-rect 等命令。")
        print("   如果确实要修改实际地图，请加 --edit-map 标记。")
        return

    # ── 修改实际地图 ──
    if has_map_ops:
        if args.backup:
            backup_pgm(MAP_PGM)

        _, data = load_pgm(MAP_PGM)
        print(f"\n修改前: ", end="")
        show_info(data, "实际地图")

        if args.add_rect:
            for rect in args.add_rect:
                data = add_obstacle_rect(data, *rect)
        if args.add_circle:
            for circle in args.add_circle:
                data = add_obstacle_circle(data, *circle)
        if args.fill_unknown:
            for rect in args.fill_unknown:
                data = fill_unknown_as_obstacle(data, *rect)
        if args.erode_free:
            data = erode_free_space(data, args.erode_free)

        print(f"\n修改后: ", end="")
        show_info(data, "实际地图")

        if args.dry_run:
            print("\n🔍 预览模式，未保存。去掉 --dry-run 以保存。")
        else:
            save_pgm(data, MAP_PGM)
            print("\n⚠️  已修改实际地图！重启 navigation 生效。定位可能受影响。")

    # ── 修改禁区蒙版（默认操作） ──
    if has_keepout_ops:
        # 获取地图尺寸以创建匹配的禁区蒙版
        _, map_data = load_pgm(MAP_PGM)
        if args.backup and Path(KEEPOUT_PGM).exists():
            backup_pgm(KEEPOUT_PGM)

        keepout = ensure_keepout_mask(map_data.shape)
        print(f"\n修改前: ", end="")
        show_info(keepout, "禁区蒙版")

        if args.add_keepout_rect:
            for rect in args.add_keepout_rect:
                keepout = add_keepout_rect(keepout, *rect)
        if args.add_keepout_circle:
            for circle in args.add_keepout_circle:
                keepout = add_keepout_circle(keepout, *circle)
        if args.clear_keepout_rect:
            for rect in args.clear_keepout_rect:
                keepout = clear_keepout_rect(keepout, *rect)
        if args.erode_keepout_free:
            keepout = erode_keepout_free(keepout, args.erode_keepout_free)

        print(f"\n修改后: ", end="")
        show_info(keepout, "禁区蒙版")

        if args.dry_run:
            print("\n🔍 预览模式，未保存。去掉 --dry-run 以保存。")
        else:
            save_pgm(keepout, KEEPOUT_PGM)
            print("\n✅ 禁区蒙版已更新！keepout_mask_publisher 会自动重新加载（1秒内）。")
            print("   路径规划将自动避开禁区，定位不受影响。")


if __name__ == "__main__":
    main()
