#!/usr/bin/env python3
"""
位置映射器 —— 将服务器返回的人类可读地址映射为导航坐标

JSON 文件格式 (locations.json):

  {
    "docking_stations": {
      "1F-充电站": { "x": 0.0, "y": 0.0, "z": 0, "desc": "..." }
    },
    "bookshelves": {
      "3F · A区 · 书架 A-04": { "x": 1.0, "y": 2.0, "z": 6, "desc": "..." }
    },
    "seats": {
      "12": { "x": 7.5, "y": 3.0, "z": 0, "desc": "..." }
    }
  }

坐标约定: x=东西向(m), y=南北向(m), z=楼层(0=1F, 3=2F, 6=3F)
"""

import json
import logging
import os
from typing import Optional

logger = logging.getLogger(__name__)


class LocationMapper:
    """地址描述 → (x, y, z) 导航坐标的映射器"""

    def __init__(self, json_path: str):
        """
        Args:
            json_path: locations.json 的绝对路径，或相对于工作目录的路径
        """
        if not os.path.exists(json_path):
            raise FileNotFoundError(
                f"位置映射文件不存在: {json_path}\n"
                f"    请确保 locations.json 已部署到包的 config/ 目录"
            )

        with open(json_path, "r", encoding="utf-8") as f:
            self._data = json.load(f)

        # 统计加载数量
        docks = len(self._data.get("docking_stations", {}))
        shelves = len(self._data.get("bookshelves", {}))
        seats = len(self._data.get("seats", {}))
        logger.info(
            f"位置映射已加载: {docks} 个充电站, "
            f"{shelves} 个书架, {seats} 个座位"
        )

    # ── 公开方法 ──────────────────────────────────────

    def lookup(self, name: str) -> Optional[dict]:
        """
        在所有分类中查找指定名称的位置。
        搜索顺序: bookshelves → seats → docking_stations

        Returns:
            dict with 'x','y','z' keys, or None
        """
        if not name:
            return None

        for section in ["bookshelves", "seats", "docking_stations"]:
            section_data = self._data.get(section, {})
            if name in section_data:
                coord = section_data[name]
                return {"x": coord["x"], "y": coord["y"], "z": coord["z"]}

        logger.warning(f"位置 '{name}' 在 locations.json 中未找到")
        return None

    def get_bookshelf(self, location_str: str) -> Optional[dict]:
        """
        查找书架坐标。
        对应服务器返回的 book_location 字段，
        如 "3F · A区 · 书架 A-04"
        """
        return self._lookup_in("bookshelves", location_str)

    def get_seat(self, table_number: str) -> Optional[dict]:
        """
        查找座位坐标。
        对应服务器返回的 table_number 字段，
        如 "12", "A-01" 等
        """
        return self._lookup_in("seats", table_number)

    def get_docking(self, station_name: str) -> Optional[dict]:
        """
        查找停靠/充电站坐标。
        如 "1F-充电站", "2F-充电站", "3F-充电站"
        """
        return self._lookup_in("docking_stations", station_name)

    # ── 辅助 ──────────────────────────────────────────

    def _lookup_in(self, section: str, name: str) -> Optional[dict]:
        """在指定分组中查找"""
        if not name:
            return None
        section_data = self._data.get(section, {})
        if name in section_data:
            coord = section_data[name]
            return {"x": coord["x"], "y": coord["y"], "z": coord["z"]}
        logger.warning(f"{section} 中未找到 '{name}'")
        return None

    def get_all_in_section(self, section: str) -> dict:
        """返回指定分组的全部数据（用于调试/列表展示）"""
        return self._data.get(section, {})
