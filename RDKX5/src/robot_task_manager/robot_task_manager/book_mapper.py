#!/usr/bin/env python3
"""
书籍映射器 —— 将服务器返回的书名映射为机械臂用的书籍编号

JSON 文件格式 (books.json):

  {
    "books": {
      "Minimalist Forms":  { "book_number": 1 },
      "Silent Spaces":     { "book_number": 2 }
    }
  }

数据流:
  服务器 book_title → BookMapper → book_number → 机械臂串口 → 返回 "0"
"""

import json
import logging
import os
from typing import Optional

logger = logging.getLogger(__name__)


class BookMapper:
    """书名 → 书籍编号 的映射器"""

    def __init__(self, json_path: str):
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"书籍映射文件不存在: {json_path}")

        with open(json_path, "r", encoding="utf-8") as f:
            self._data = json.load(f)

        count = len(self._data.get("books", {}))
        logger.info(f"书籍映射已加载: {count} 本书")

    def get_book_number(self, book_title: str) -> Optional[int]:
        """
        根据书名查找书籍编号。

        Args:
            book_title: 服务器返回的书名，如 "Minimalist Forms"

        Returns:
            书籍编号 (int)，找不到时返回 None
        """
        if not book_title:
            return None

        book = self._data.get("books", {}).get(book_title)
        if book is None:
            logger.warning(f"书名 '{book_title}' 在 books.json 中未找到")
            return None

        return book.get("book_number")

    def get_book_title(self, book_number: int) -> Optional[str]:
        """根据编号反查书名（调试用）"""
        for title, info in self._data.get("books", {}).items():
            if info.get("book_number") == book_number:
                return title
        return None
