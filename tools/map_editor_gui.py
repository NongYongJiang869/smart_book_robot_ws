#!/usr/bin/env python3
"""
Interactive keepout zone editor — draw no-go zones without modifying the SLAM map.

The actual map (library_map.pgm) is displayed as a grayscale background for
reference.  Editing operations write to a separate *keepout mask*
(library_keepout_mask.pgm).  Nav2's KeepoutFilter overlays the keepout mask
onto the global costmap so path planning avoids marked areas while AMCL
localisation continues to use the clean original map.

Usage:
    python3 tools/map_editor_gui.py

Controls:
    Toolbar buttons → Pan / Zoom (default mode, works normally)
    Press D         → Toggle Draw mode
                       L-drag = mark keepout zone (shown in RED)
                       R-drag = clear keepout zone (restore free)
    Press Ctrl+S    → Save keepout mask
    Press S (或 Ctrl+S) → 保存禁区蒙版
    Press Z         → Undo last edit
    Press Q / ESC   → Quit
    Press E         → Erode free space edges (close all narrow gaps)
"""

import os
import sys
from datetime import datetime

import numpy as np
from PIL import Image
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import RectangleSelector
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk

MAP_PGM    = "/root/library_map.pgm"
KEEPOUT_PGM = "/root/library_keepout_mask.pgm"
RESOLUTION  = 0.05
ORIGIN_X, ORIGIN_Y = -20, -20

undo_stack = []


def world_to_pixel(wx, wy):
    return int((wx - ORIGIN_X) / RESOLUTION), int((wy - ORIGIN_Y) / RESOLUTION)


def ensure_keepout_mask(ref_shape):
    """Return keepout mask data, creating a blank one if it doesn't exist."""
    if os.path.exists(KEEPOUT_PGM):
        img = Image.open(KEEPOUT_PGM)
        data = np.array(img).copy()
        if data.shape == ref_shape:
            return data
        print(f"Keepout mask shape {data.shape} != map shape {ref_shape}, re-creating...")

    data = np.full(ref_shape, 254, dtype=np.uint8)
    Image.fromarray(data).save(KEEPOUT_PGM)
    print(f"Created blank keepout mask: {KEEPOUT_PGM}")
    return data


def make_overlay(keepout, height, width, extent):
    """Build an RGBA overlay showing keepout zones in red."""
    overlay = np.zeros((height, width, 4), dtype=np.uint8)
    keepout_mask = keepout <= 89   # occupied cells
    overlay[keepout_mask, 0] = 200  # R
    overlay[keepout_mask, 1] = 30   # G
    overlay[keepout_mask, 2] = 30   # B
    overlay[keepout_mask, 3] = 150  # alpha
    return overlay


class MapEditor:
    def __init__(self, map_pgm):
        self.map_pgm = map_pgm

        # ── load the REAL map (display only, never modified) ──
        map_img = Image.open(map_pgm)
        self.map_data = np.array(map_img).copy()
        self.height, self.width = self.map_data.shape

        # ── load or create the KEEPOUT mask (this is what we edit) ──
        self.keepout = ensure_keepout_mask((self.height, self.width))

        self.draw_mode = False
        self._build_extent()

        # ── Window + toolbar ──
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.fig.canvas.manager.set_window_title(
            "Keepout Editor | Toolbar=Pan/Zoom | D=Draw | Ctrl+S=Save | Z=Undo | Q=Quit"
        )
        self.toolbar = NavigationToolbar2Tk(
            self.fig.canvas, self.fig.canvas.manager.window
        )
        self.toolbar.update()

        # Override toolbar's default 's' key: redirect save-figure to our keepout save
        self._toolbar_save_orig = self.toolbar.save_figure
        self.toolbar.save_figure = self._toolbar_save_wrapper

        # Layer 1 — real map (grayscale, READ-ONLY)
        self.map_im = self.ax.imshow(
            self.map_data, origin='lower', extent=self.extent,
            cmap='gray', vmin=0, vmax=255,
        )

        # Layer 2 — keepout overlay (red where restricted)
        self._update_overlay()
        self.overlay_im = self.ax.imshow(
            self.overlay_rgba, origin='lower', extent=self.extent,
        )

        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title(
            "Toolbar=Pan/Zoom | D=Draw mode (L=add keepout  R=clear) | S=save Z=undo Q=quit"
        )
        self.ax.grid(True, alpha=0.2)

        # ── Rectangle selectors ──
        self.rect_sel = RectangleSelector(
            self.ax, self._on_mark_keepout,
            useblit=True, button=[1],
            props=dict(facecolor='red', edgecolor='red', alpha=0.4, fill=True),
            interactive=False,
        )
        self.clear_sel = RectangleSelector(
            self.ax, self._on_clear_keepout,
            useblit=True, button=[3],
            props=dict(facecolor='green', edgecolor='green', alpha=0.4, fill=True),
            interactive=False,
        )
        self.rect_sel.set_active(False)
        self.clear_sel.set_active(False)

        # ── Status text ──
        self.status_text = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes,
            va='top', fontsize=10, color='white',
            bbox=dict(boxstyle='round', facecolor='black', alpha=0.7),
            family='monospace',
        )

        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self._update_status()
        self._redraw()

    # ── helpers ────────────────────────────────────────────────
    def _build_extent(self):
        self.extent = [
            ORIGIN_X, ORIGIN_X + self.width * RESOLUTION,
            ORIGIN_Y, ORIGIN_Y + self.height * RESOLUTION,
        ]

    def _update_overlay(self):
        self.overlay_rgba = make_overlay(self.keepout, self.height, self.width, self.extent)

    def _redraw(self):
        self._update_overlay()
        self.overlay_im.set_data(self.overlay_rgba)
        self.fig.canvas.draw_idle()

    def _update_status(self):
        occupied = (self.keepout <= 89).sum()
        free = (self.keepout >= 229).sum()
        total = self.keepout.size
        mode = "[DRAW]" if self.draw_mode else "[PAN/ZOOM]"
        pct = occupied / total * 100
        self.status_text.set_text(
            f"{mode}  KEEPOUT:{occupied} cells ({pct:.1f}% of map)  "
            f"FREE:{free/total*100:.1f}%  |  S-save Z-undo Q-quit"
        )

    # ── world → pixel ──────────────────────────────────────────
    def _world_to_px_bounds(self, x1, y1, x2, y2):
        px1, py1 = world_to_pixel(x1, y1)
        px2, py2 = world_to_pixel(x2, y2)
        x_min, x_max = sorted([px1, px2])
        y_min, y_max = sorted([py1, py2])
        x_min = max(0, x_min); x_max = min(self.width - 1, x_max)
        y_min = max(0, y_min); y_max = min(self.height - 1, y_max)
        return x_min, x_max, y_min, y_max

    def _push_undo(self):
        undo_stack.append(self.keepout.copy())
        if len(undo_stack) > 50:
            undo_stack.pop(0)

    # ── toggle draw mode ───────────────────────────────────────
    def _toolbar_save_wrapper(self, *args, **kwargs):
        """Intercept toolbar's 's' key → save keepout mask instead of figure."""
        self._save()
        return '/root/library_keepout_mask.pgm'  # dummy return path

    def toggle_draw_mode(self):
        self.draw_mode = not self.draw_mode
        self.rect_sel.set_active(self.draw_mode)
        self.clear_sel.set_active(self.draw_mode)
        if self.draw_mode:
            self.ax.set_title(
                "*** DRAW MODE ***  L-drag=add keepout  R-drag=clear  D=exit"
            )
        else:
            self.ax.set_title(
                "PAN/ZOOM mode (toolbar works)  |  D=draw  S=save  Q=quit"
            )
        self._update_status()
        self._redraw()

    # ── draw callbacks (modify keepout mask ONLY) ──────────────
    def _on_mark_keepout(self, eclick, erelease):
        """Left-drag: mark area as keepout (black in mask = occupied)."""
        x1, y1 = eclick.xdata, eclick.ydata
        x2, y2 = erelease.xdata, erelease.ydata
        if None in (x1, y1, x2, y2):
            return
        self._push_undo()
        x_min, x_max, y_min, y_max = self._world_to_px_bounds(x1, y1, x2, y2)
        self.keepout[y_min:y_max+1, x_min:x_max+1] = 0  # black → keepout
        print(f"  +keepout: ({x1:.2f},{y1:.2f})→({x2:.2f},{y2:.2f})  "
              f"px[{x_min},{y_min}]→[{x_max},{y_max}]")
        self._update_status()
        self._redraw()

    def _on_clear_keepout(self, eclick, erelease):
        """Right-drag: clear keepout zone (restore to free = white)."""
        x1, y1 = eclick.xdata, eclick.ydata
        x2, y2 = erelease.xdata, erelease.ydata
        if None in (x1, y1, x2, y2):
            return
        self._push_undo()
        x_min, x_max, y_min, y_max = self._world_to_px_bounds(x1, y1, x2, y2)
        self.keepout[y_min:y_max+1, x_min:x_max+1] = 254  # white → free
        print(f"  -keepout: ({x1:.2f},{y1:.2f})→({x2:.2f},{y2:.2f})  "
              f"px[{x_min},{y_min}]→[{x_max},{y_max}]")
        self._update_status()
        self._redraw()

    # ── key handlers ───────────────────────────────────────────
    def _on_key(self, event):
        if event.key == 'd':
            self.toggle_draw_mode()
        elif event.key in ('q', 'escape'):
            plt.close()
            print("Quit (not saved)")
        elif event.key in ('s', 'ctrl+s'):
            self._save()
        elif event.key == 'z':
            if undo_stack:
                self.keepout = undo_stack.pop()
                print("Undo (keepout)")
                self._update_status()
                self._redraw()
            else:
                print("Nothing to undo")
        elif event.key == 'e':
            self._erode_free()

    def _erode_free(self):
        """Erode free-space edges in keepout mask (close narrow gaps)."""
        from scipy.ndimage import binary_erosion
        try:
            cells = int(input(
                "Erode by how many cells? (1 cell = 5cm, suggest 3-5): "
            ))
        except (ValueError, EOFError):
            return
        self._push_undo()
        free_mask = self.keepout >= 229
        eroded = binary_erosion(free_mask, iterations=cells)
        filled = ~eroded & free_mask
        self.keepout[filled] = 0
        print(f"  Eroded {cells} cells ({cells*RESOLUTION:.2f}m), "
              f"closed {filled.sum()} px in keepout mask")
        self._update_status()
        self._redraw()

    def _save(self):
        """Save the keepout mask PGM. The real map is NEVER modified."""
        # Backup previous keepout mask
        if os.path.exists(KEEPOUT_PGM):
            backup = KEEPOUT_PGM.replace('.pgm', f'.backup-{datetime.now():%m%d-%H%M}.pgm')
            Image.open(KEEPOUT_PGM).save(backup)
            print(f"Backup: {backup}")

        Image.fromarray(self.keepout).save(KEEPOUT_PGM)

        occupied = (self.keepout <= 89).sum()
        total = self.keepout.size
        print(f"✅ Saved: {KEEPOUT_PGM}")
        print(f"   Keepout zones: {occupied} cells ({occupied/total*100:.1f}% of map)")
        print("   Navigation will auto-pick-up within 1 second.")

        # Flash window title as visual feedback
        self.fig.canvas.manager.set_window_title(
            f"✅ SAVED — {occupied} keepout cells ({occupied/total*100:.1f}%) — Ctrl+S"
        )

        self._update_status()


def main():
    print("=== Keepout Zone Editor ===")
    print(f"Real map (read-only):  {MAP_PGM}")
    print(f"Keepout mask (editing): {KEEPOUT_PGM}")
    print()
    print("Toolbar = Pan / Zoom (default)")
    print("D = Toggle Draw mode")
    print("    Left-drag  = mark keepout zone (red)")
    print("    Right-drag = clear keepout zone (green)")
    print("S = Save keepout mask")
    print("Z = Undo last edit")
    print("E = Erode free edges (close gaps in keepout mask)")
    print("Q / ESC = Quit")
    print()
    MapEditor(MAP_PGM)
    plt.show()


if __name__ == "__main__":
    main()
