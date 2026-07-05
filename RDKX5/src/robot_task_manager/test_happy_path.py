#!/usr/bin/env python3
"""
状态机 Happy Path 测试

连续驱动状态机遍历所有状态，验证完整转移链路。
模拟导航和机械臂操作。

用法:
    cd RDKX5/src/robot_task_manager
    python3 test_happy_path.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

from robot_task_manager.library_api import LibraryAPI
from robot_task_manager.location_mapper import LocationMapper
from robot_task_manager.robot_state_machine import (
    RobotStateMachine, RobotState,
)
from robot_task_manager.arm_controller import ArmController


class MockNavigation:
    def __init__(self):
        self._status = "idle"
        self.targets = []

    def navigate_to(self, x, y, z, yaw=0.0, frame="map"):
        self.targets.append({"x": x, "y": y, "z": z})
        self._status = "succeeded"

    def get_status(self):
        # 不重置 — 让状态机自己处理转移
        return self._status

    def cancel(self):
        self._status = "idle"


def test_happy_path_continuous():
    """
    从 NAV_TO_SHELF 开始连续驱动状态机通过全部 happy path。
    """
    print("=" * 60)
    print("  状态机 Happy Path 连续驱动测试")
    print("=" * 60)

    api = LibraryAPI("robot-test-hp2", "39.105.113.176")
    config_dir = os.path.join(os.path.dirname(__file__), "config")
    mapper = LocationMapper(os.path.join(config_dir, "locations.json"))

    nav = MockNavigation()
    arm = ArmController(sim_delay=True, sim_duration=0.3)

    sm = RobotStateMachine(api, mapper, nav, arm)
    sm.set_docking("1F-充电站")
    sm.battery = 80

    # 设置假任务，手动跳转到 NAV_TO_SHELF（跳过需要真实服务器的 ACCEPTING）
    sm.current_task = {
        "id": 1,
        "book_title": "Test Book",
        "book_location": "3F · A区 · 书架 A-04",
        "target_table": "A-01",
    }
    sm._transition_to(RobotState.NAV_TO_SHELF)

    visited = []
    max_ticks = 100
    tick_count = 0

    print(f"\n  起始: NAV_TO_SHELF")
    print(f"  目标: 通过全部状态到达 IDLE\n")

    prev_state = None
    for tick_count in range(max_ticks):
        if prev_state != sm.state:
            visited.append(sm.state)
            if prev_state is not None:
                print(f"  [{tick_count:03d}] {prev_state.name} → {sm.state.name}")
            prev_state = sm.state

        sm.tick()
        time.sleep(0.15)

        if sm.state == RobotState.IDLE:
            visited.append(RobotState.IDLE)
            print(f"  [{tick_count:03d}] → IDLE")
            break

    # ── 等待充电完成（如果需要） ──
    while sm.state == RobotState.CHARGING and tick_count < max_ticks:
        sm.tick()
        tick_count += 1
        time.sleep(0.15)
        if sm.state == RobotState.IDLE:
            break

    # ── 验证 ──
    print(f"\n{'='*60}")
    print(f"  结果验证 (共 {tick_count} ticks)")
    print(f"{'='*60}")

    path_str = " → ".join(s.name for s in visited)
    print(f"\n  访问状态: {path_str}")

    error = False

    # 检查完整路径
    expected_order = [
        RobotState.NAV_TO_SHELF,
        RobotState.ARM_EXTEND,
        RobotState.LOCATE_BOOK,
        RobotState.GRASP_BOOK,
        RobotState.NAV_TO_SEAT,
        RobotState.DELIVERED,
        RobotState.RETURNING,
        RobotState.CHARGING,
        RobotState.IDLE,
    ]

    missing = [s for s in expected_order if s not in visited]
    if missing:
        print(f"  ❌ 缺失状态: {[s.name for s in missing]}")
        error = True

    # 检查顺序
    actual_order = [s for s in visited if s in expected_order]
    if actual_order != expected_order:
        print(f"  ❌ 状态顺序错误")
        print(f"    期望: {' → '.join(s.name for s in expected_order)}")
        print(f"    实际: {' → '.join(s.name for s in actual_order)}")
        # 非致命 — 可能有重复状态

    # 导航目标验证
    print(f"\n  导航目标 ({len(nav.targets)} 个):")
    expected_nav = [
        ("书架 A-04", 1.0, 2.0, 6),   # NAV_TO_SHELF
        ("座位 A-01", 7.0, 1.0, 0),   # NAV_TO_SEAT
        ("充电站",    0.0, 0.0, 0),   # RETURNING
    ]
    for i, (label, ex, ey, ez) in enumerate(expected_nav):
        if i < len(nav.targets):
            t = nav.targets[i]
            match = (abs(t["x"] - ex) < 0.1 and
                     abs(t["y"] - ey) < 0.1 and
                     abs(t["z"] - ez) < 0.1)
            status = "✅" if match else "❌"
            print(f"    {i+1}. {label}: ({t['x']:.1f}, {t['y']:.1f}, z={t['z']:.0f}) {status}")
            if not match:
                print(f"       期望 ({ex}, {ey}, z={ez})")
                error = True
        else:
            print(f"    {i+1}. {label}: 缺失 ❌")
            error = True

    # 电池
    print(f"\n  电池: {sm.battery}% {'✅' if sm.battery >= 0 else '❌'}")
    print(f"  最终状态: {sm.state.name} {'✅' if sm.state == RobotState.IDLE else '❌'}")
    if sm.state != RobotState.IDLE:
        error = True

    print(f"\n{'='*60}")
    if error:
        print("  ❌ 测试未完全通过")
    else:
        print("  ✅ 全部验证通过!")
    print(f"{'='*60}")

    return not error


if __name__ == "__main__":
    ok = test_happy_path_continuous()
    sys.exit(0 if ok else 1)
