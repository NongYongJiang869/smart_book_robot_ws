#!/usr/bin/env python3
"""
机器人任务状态机 —— 管理取书全流程的状态转移

状态转移图:

  IDLE ──[发现任务]──→ ACCEPTING ──[接单成功]──→ NAV_TO_SHELF
    ↑                    │                            │
    │                    └──[接单失败]──→ IDLE         │
    │                                                  ↓
  CHARGING ←── RETURNING ←── DELIVERED ←── NAV_TO_SEAT
    │              ↑                          ↑
    │              │                          │
    └── IDLE ──────┘    GRASP_BOOK ←── LOCATE_BOOK ←── ARM_EXTEND

  任何状态 ──[异常]──→ ERROR ──[恢复]──→ IDLE
"""

import logging
import time
from enum import Enum, auto
from typing import Optional

from .library_api import (
    LibraryAPI,
    STATUS_ACCEPTED, STATUS_SEARCHING, STATUS_ARM_EXTENDING,
    STATUS_LOCATING, STATUS_GRASPING, STATUS_DELIVERING, STATUS_DELIVERED,
    ROBOT_IDLE, ROBOT_BUSY, ROBOT_CHARGING, ROBOT_ERROR,
)
from .location_mapper import LocationMapper

logger = logging.getLogger(__name__)


# ── 状态枚举 ──────────────────────────────────────────

class RobotState(Enum):
    IDLE          = auto()   # 空闲，等待轮询任务
    ACCEPTING     = auto()   # 正在接单
    NAV_TO_SHELF  = auto()   # 导航到书架（searching）
    ARM_EXTEND    = auto()   # 机械臂展开（arm_extending）
    LOCATE_BOOK   = auto()   # 视觉定位书籍（locating）
    GRASP_BOOK    = auto()   # 夹取书籍（grasping）
    NAV_TO_SEAT   = auto()   # 运送至座位（delivering）
    DELIVERED     = auto()   # 已送达（delivered）
    RETURNING     = auto()   # 返回停靠站
    CHARGING      = auto()   # 充电中
    ERROR         = auto()   # 故障/异常


# ── 状态机 ────────────────────────────────────────────

class RobotStateMachine:
    """机器人任务状态机"""

    def __init__(self, api: LibraryAPI, mapper: LocationMapper,
                 nav, arm):
        """
        Args:
            api:    LibraryAPI 实例
            mapper: LocationMapper 实例
            nav:    NavigationController 实例 (需实现 navigate_to / status / cancel)
            arm:    ArmController 实例 (需实现 extend / locate / grasp / retract)
        """
        self.api = api
        self.mapper = mapper
        self.nav = nav
        self.arm = arm

        # ── 当前状态 ──
        self.state = RobotState.IDLE
        self._state_entered_at = time.time()   # 进入当前状态的时间
        self._first_tick = True                 # 是否是该状态的第一个 tick

        # ── 任务上下文 ──
        self.current_task: Optional[dict] = None
        self.error_reason: str = ""

        # ── 重试计数 ──
        self._nav_retries = 0
        self._max_nav_retries = 2

        # ── 停靠站 ──
        self.docking_station = "1F-充电站"
        self.docking_coord: Optional[dict] = None

        # ── 电池 ──
        self.battery = 100

        # ── 回调（供 ROS 节点注册） ──
        self.on_state_changed = None  # callable(old, new)

    # ── 公共接口 ──────────────────────────────────────

    @property
    def state_name(self) -> str:
        return self.state.name.lower()

    @property
    def elapsed_in_state(self) -> float:
        """在当前状态已停留的秒数"""
        return time.time() - self._state_entered_at

    def set_docking(self, station_name: str):
        """设置停靠站名称并查找坐标"""
        self.docking_station = station_name
        self.docking_coord = self.mapper.get_docking(station_name)
        if self.docking_coord is None:
            logger.warning(f"停靠站 '{station_name}' 坐标未找到，使用 (0,0,0)")
            self.docking_coord = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}

    def assign_task(self, task: dict):
        """分配新任务（由 ROS 节点在发现 pending 任务时调用）"""
        self.current_task = task
        self._transition_to(RobotState.ACCEPTING)

    def tick(self):
        """
        每个周期调用一次，执行当前状态的逻辑。
        由 ROS2 定时器驱动 (建议 5Hz = 0.2s)。
        """
        handler_name = f'_handle_{self.state.name.lower()}'
        handler = getattr(self, handler_name, None)
        if handler is None:
            logger.error(f"状态 {self.state.name} 没有对应的 handler: {handler_name}")
            self._transition_to(RobotState.ERROR)
            return

        old_state = self.state
        try:
            handler()
        except Exception as e:
            logger.exception(f"状态 {self.state.name} handler 异常: {e}")
            self.error_reason = str(e)
            self._transition_to(RobotState.ERROR)

        # 只有未发生转移时才清除 _first_tick，
        # 否则新状态需要 _first_tick=True 来执行 entry 动作
        if self.state == old_state:
            self._first_tick = False

    # ── 状态转移 ──────────────────────────────────────

    def _transition_to(self, new_state: RobotState):
        """执行状态转移"""
        if new_state == self.state:
            return
        old = self.state
        logger.info(f"状态转移: {old.name} → {new_state.name}")
        self.state = new_state
        self._state_entered_at = time.time()
        self._first_tick = True

        # 调用回调
        if self.on_state_changed is not None:
            try:
                self.on_state_changed(old, new_state)
            except Exception:
                pass

    # ── 状态处理器 ────────────────────────────────────

    # ── IDLE ──
    def _handle_idle(self):
        """空闲状态：不做任何事，等待外部分配任务"""
        pass  # 由 ROS 节点的 poll_loop 检测新任务并调用 assign_task()

    # ── ACCEPTING ──
    def _handle_accepting(self):
        """接单状态：调用服务器 accept API"""
        if self.current_task is None:
            logger.error("ACCEPTING 但没有 current_task")
            self._transition_to(RobotState.IDLE)
            return

        order_id = self.current_task["id"]
        resp = self.api.accept_task(order_id)

        if resp is None:
            self.error_reason = f"接单 #{order_id} 失败: 网络错误"
            self._transition_to(RobotState.ERROR)
            return

        if not resp.get("accepted"):
            # 409 冲突或其他 — 任务已被抢
            logger.warning(f"接单 #{order_id} 被拒绝: {resp}")
            self.current_task = None
            self._transition_to(RobotState.IDLE)
            return

        # 接单成功，更新任务信息
        task = resp.get("task", {})
        self.current_task["book_location"] = task.get("book_location", "")
        self.current_task["target_table"] = task.get("target_table", "")
        logger.info(
            f"接单成功 #{order_id}: 《{task.get('book_title', '?')}》 "
            f"书架={task.get('book_location')} 座位={task.get('target_table')}"
        )
        self._transition_to(RobotState.NAV_TO_SHELF)

    # ── NAV_TO_SHELF ──
    def _handle_nav_to_shelf(self):
        """导航到书架（优先使用预定义路径途经点）"""
        if self._first_tick:
            # 查找书架坐标
            location = self.current_task.get("book_location", "")
            coord = self.mapper.get_bookshelf(location)
            if coord is None:
                self.error_reason = f"书架坐标未找到: '{location}'"
                self._transition_to(RobotState.ERROR)
                return

            self._update_server_status(STATUS_SEARCHING, location)
            self._nav_retries = 0

            # 尝试查找预定义路径（途经点序列）
            path = self.mapper.find_path(self.docking_station, location)
            if path:
                logger.info(f"开始导航到书架 (途经 {len(path)} 个点): {location}")
                self.nav.navigate_waypoints(path)
            else:
                logger.info(f"开始导航到书架 (直达): {location} → ({coord['x']:.1f}, {coord['y']:.1f})")
                self.nav.navigate_to(coord["x"], coord["y"], coord["z"],
                                     coord.get("yaw", 0.0))

        # 检查导航状态
        status = self.nav.get_status()
        if status == "succeeded":
            logger.info("已到达书架")
            self._transition_to(RobotState.ARM_EXTEND)
        elif status == "failed":
            self._nav_retries += 1
            if self._nav_retries <= self._max_nav_retries:
                logger.warning(f"导航到书架失败，重试 {self._nav_retries}/{self._max_nav_retries} (直达模式)")
                coord = self.mapper.get_bookshelf(
                    self.current_task.get("book_location", ""))
                if coord:
                    self.nav.navigate_to(coord["x"], coord["y"], coord["z"],
                                 coord.get("yaw", 0.0))
            else:
                self.error_reason = "导航到书架失败（已达最大重试次数）"
                self._transition_to(RobotState.ERROR)

    # ── ARM_EXTEND ──
    def _handle_arm_extend(self):
        """展开机械臂"""
        if self._first_tick:
            self._update_server_status(STATUS_ARM_EXTENDING)
            self.arm.extend()
            logger.info("机械臂正在展开...")

        status = self.arm.get_status()
        if status == "succeeded":
            logger.info("机械臂展开完成")
            self._transition_to(RobotState.LOCATE_BOOK)
        elif status == "failed":
            self.error_reason = "机械臂展开失败"
            self._transition_to(RobotState.ERROR)

    # ── LOCATE_BOOK ──
    def _handle_locate_book(self):
        """视觉定位书籍"""
        if self._first_tick:
            self._update_server_status(STATUS_LOCATING)
            book_title = self.current_task.get("book_title", "")
            self.arm.locate_book(book_title)
            logger.info(f"视觉定位中: 《{book_title}》")

        status = self.arm.get_status()
        if status == "succeeded":
            logger.info("书籍定位成功")
            self._transition_to(RobotState.GRASP_BOOK)
        elif status == "failed":
            self.error_reason = "书籍定位失败"
            self._transition_to(RobotState.ERROR)

    # ── GRASP_BOOK ──
    def _handle_grasp_book(self):
        """夹取书籍"""
        if self._first_tick:
            self._update_server_status(STATUS_GRASPING)
            self.arm.grasp()
            logger.info("正在夹取书籍...")

        status = self.arm.get_status()
        if status == "succeeded":
            logger.info("书籍夹取成功")
            self._transition_to(RobotState.NAV_TO_SEAT)
        elif status == "failed":
            self.error_reason = "书籍夹取失败"
            self._transition_to(RobotState.ERROR)

    # ── NAV_TO_SEAT ──
    def _handle_nav_to_seat(self):
        """运送至读者座位"""
        if self._first_tick:
            table_num = self.current_task.get("target_table", "")
            coord = self.mapper.get_seat(table_num)
            if coord is None:
                self.error_reason = f"座位坐标未找到: '{table_num}'"
                self._transition_to(RobotState.ERROR)
                return
            logger.info(f"开始运送至座位: {table_num} → ({coord['x']:.1f}, {coord['y']:.1f}, z={coord['z']})")
            self._update_server_status(STATUS_DELIVERING, table_num)
            self._nav_retries = 0
            self.nav.navigate_to(coord["x"], coord["y"], coord["z"],
                                 coord.get("yaw", 0.0))

        status = self.nav.get_status()
        if status == "succeeded":
            logger.info("已到达座位")
            self._transition_to(RobotState.DELIVERED)
        elif status == "failed":
            self._nav_retries += 1
            if self._nav_retries <= self._max_nav_retries:
                logger.warning(f"导航到座位失败，重试 {self._nav_retries}/{self._max_nav_retries}")
                coord = self.mapper.get_seat(
                    self.current_task.get("target_table", ""))
                if coord:
                    self.nav.navigate_to(coord["x"], coord["y"], coord["z"],
                                 coord.get("yaw", 0.0))
            else:
                self.error_reason = "导航到座位失败（已达最大重试次数）"
                self._transition_to(RobotState.ERROR)

    # ── DELIVERED ──
    def _handle_delivered(self):
        """
        已送达座位：
          1. 等 rotate_to_goal 转正方向 + 停稳 (3s)
          2. 发送放书指令，等机械臂返回 "6"
          3. 返回充电站
        """
        if self._first_tick:
            order_id = self.current_task.get("id", 0)
            table_num = self.current_task.get("target_table", "")
            self._update_server_status(STATUS_DELIVERED, table_num)
            logger.info(f"📦 任务 #{order_id} 已送达!")
            self.battery = max(10, self.battery - self._estimate_consumption())
            self._phase = 'stabilize'  # 阶段：stabilize → placing → done

        # 阶段 1: 等旋转完成 + 停稳 3 秒
        if self._phase == 'stabilize':
            if self.elapsed_in_state < 3.0:
                return
            # 触发放书
            self.arm.place()
            logger.info("🦾 发送放书指令...")
            self._phase = 'placing'
            return

        # 阶段 2: 等机械臂放书完成（收到 "6"）
        if self._phase == 'placing':
            arm_status = self.arm.get_status()
            if arm_status == "active":
                return
            if arm_status == "failed":
                self.error_reason = "放书失败"
                self._transition_to(RobotState.ERROR)
                return
            logger.info("✅ 放书完成")
            self._phase = 'done'
            return

        # 阶段 3: 返回
        if self._phase == 'done':
            self._transition_to(RobotState.RETURNING)

    # ── RETURNING ──
    def _handle_returning(self):
        """返回停靠站（优先使用预定义路径）"""
        if self._first_tick:
            if self.docking_coord is None:
                self.set_docking(self.docking_station)

            self._nav_retries = 0

            # 尝试查找返回路径（从书架或座位到充电站）
            book_location = self.current_task.get("book_location", "")
            table_num = self.current_task.get("target_table", "")
            path = (self.mapper.find_path(book_location, self.docking_station) or
                    self.mapper.find_path(table_num, self.docking_station))

            if path:
                logger.info(f"返回停靠站 (途经 {len(path)} 个点): {self.docking_station}")
                self.nav.navigate_waypoints(path)
            else:
                logger.info(f"返回停靠站 (直达): {self.docking_station} → "
                            f"({self.docking_coord['x']:.1f}, {self.docking_coord['y']:.1f})")
                self.nav.navigate_to(
                    self.docking_coord["x"],
                    self.docking_coord["y"],
                    self.docking_coord["z"],
                    self.docking_coord.get("yaw", 0.0),
                )

        status = self.nav.get_status()
        if status == "succeeded":
            logger.info("已到达停靠站")
            self.current_task = None
            self._transition_to(RobotState.CHARGING)
        elif status == "failed":
            self._nav_retries += 1
            if self._nav_retries <= self._max_nav_retries:
                logger.warning(f"返回停靠站失败，重试 {self._nav_retries}/{self._max_nav_retries} (直达模式)")
                if self.docking_coord:
                    self.nav.navigate_to(
                        self.docking_coord["x"],
                        self.docking_coord["y"],
                        self.docking_coord["z"],
                        self.docking_coord.get("yaw", 0.0),
                    )
            else:
                self.error_reason = "返回停靠站失败（已达最大重试次数）"
                self._transition_to(RobotState.ERROR)

    # ── CHARGING ──
    def _handle_charging(self):
        """充电"""
        if self._first_tick:
            logger.info(f"🔋 开始充电 (当前电量 {self.battery}%)")

        # 模拟充电
        self.battery = min(100, self.battery + 1)

        if self.battery >= 95:
            logger.info(f"充电完成 (电量 {self.battery}%)")
            self._transition_to(RobotState.IDLE)

    # ── ERROR ──
    def _handle_error(self):
        """故障状态：等待恢复"""
        if self._first_tick:
            logger.error(f"⚠️ 进入 ERROR 状态: {self.error_reason}")
            self._update_server_status(ROBOT_ERROR)
            self.nav.cancel()

        # 停留 5 秒后尝试恢复到 IDLE
        if self.elapsed_in_state > 5.0:
            logger.info("尝试从 ERROR 恢复 → IDLE")
            self._transition_to(RobotState.IDLE)

    # ── 辅助方法 ──────────────────────────────────────

    def _update_server_status(self, status: str, position: str = ""):
        """更新服务器任务状态（后台执行，不阻塞）"""
        order_id = self.current_task.get("id") if self.current_task else None
        if order_id is None:
            # 尝试仅发心跳
            return

        resp = self.api.update_task(order_id, status, position)
        if resp is None:
            logger.warning(f"更新任务 #{order_id} 状态为 '{status}' 失败")

    def _estimate_consumption(self) -> int:
        """估算一个任务周期的电量消耗"""
        return 5  # 每次任务消耗约 5%

    def get_robot_status(self) -> str:
        """获取机器人状态码（用于心跳上报）"""
        if self.state == RobotState.IDLE:
            return ROBOT_IDLE
        elif self.state == RobotState.CHARGING:
            return ROBOT_CHARGING
        elif self.state == RobotState.ERROR:
            return ROBOT_ERROR
        else:
            return ROBOT_BUSY
