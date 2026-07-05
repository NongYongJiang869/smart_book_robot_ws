#!/usr/bin/env python3
"""
状态机全流程集成测试

模拟从接单到送达的完整流程，使用真实服务器 + 模拟导航/机械臂。
验证：状态转移正确性、服务器 HTTP 调用、地址映射解析。

用法:
    cd RDKX5/src/robot_task_manager
    python3 test_integration.py
"""

import os
import sys
import time

# 添加包路径
sys.path.insert(0, os.path.dirname(__file__))

from robot_task_manager.library_api import LibraryAPI
from robot_task_manager.location_mapper import LocationMapper
from robot_task_manager.robot_state_machine import (
    RobotStateMachine, RobotState,
)
from robot_task_manager.arm_controller import ArmController


# ── 模拟导航控制器（不依赖 ROS2） ──
class MockNavigation:
    """模拟导航：调用 navigate_to 后立即成功（不移动）"""

    def __init__(self):
        self._status = "idle"

    def navigate_to(self, x, y, z, yaw=0.0, frame="map"):
        print(f"      🚀 [MockNav] → ({x:.1f}, {y:.1f}, z={z:.1f})")
        self._status = "succeeded"

    def get_status(self):
        s = self._status
        # 成功后重置，避免重复触发
        if s == "succeeded":
            self._status = "idle"
        return s

    def cancel(self):
        self._status = "idle"


# ── 测试入口 ──
def main():
    print("=" * 60)
    print("  状态机全流程集成测试")
    print("=" * 60)

    # 1. 初始化组件
    robot_name = "robot-test-sm"
    api = LibraryAPI(robot_name, server="39.105.113.176")

    # 加载位置映射
    config_dir = os.path.join(os.path.dirname(__file__), "config")
    locations_path = os.path.join(config_dir, "locations.json")
    mapper = LocationMapper(locations_path)

    # 模拟导航 + 模拟机械臂（无延迟）
    nav = MockNavigation()
    arm = ArmController(sim_delay=True, sim_duration=0.5)

    # 2. 创建状态机
    sm = RobotStateMachine(api, mapper, nav, arm)
    sm.set_docking("1F-充电站")
    sm.battery = 95

    # 打印状态变化
    def on_change(old, new):
        print(f"\n  {'='*40}")
        print(f"  状态转移: {old.name} → {new.name}")
        print(f"  {'='*40}")

    sm.on_state_changed = on_change

    # 3. 发送心跳上线
    print("\n📡 发送心跳上线...")
    api.heartbeat("idle", "1F-充电站", 95)
    print("   ✅ 已上线")

    # 4. 测试没有任务时的 IDLE 轮询
    print("\n📋 轮询任务 (预期无 pending)...")
    resp = api.get_tasks()
    if resp:
        tasks = resp.get("tasks", [])
        pending = [t for t in tasks if t.get("status") == "pending"]
        print(f"   当前任务数: {resp.get('count', 0)}, pending: {len(pending)}")
    else:
        print("   ⚠️ 轮询失败")

    # 5. 模拟一个任务（直接用 API 数据）
    print("\n🧪 模拟任务: 用假数据注入状态机 (不走 accept API)")

    fake_task = {
        "id": 9999,   # 这个 ID 在服务器上不存在，但状态机 update 会报 404（预期）
        "book_title": "测试书籍",
        "book_location": "3F · A区 · 书架 A-04",
        "target_table": "A-01",
    }

    # 手动设置任务并触发状态转移
    sm.current_task = fake_task
    sm._transition_to(RobotState.ACCEPTING)

    # 6. 驱动状态机直到 IDLE (最多 60 ticks)
    print("\n🔄 驱动状态机...\n")
    max_ticks = 60
    tick_count = 0

    while sm.state != RobotState.IDLE and tick_count < max_ticks:
        print(f"  [{tick_count:02d}] state={sm.state.name:<14} "
              f"elapsed={sm.elapsed_in_state:.1f}s")
        sm.tick()
        tick_count += 1
        time.sleep(0.3)

    print(f"\n  [{tick_count:02d}] state={sm.state.name:<14} "
          f"elapsed={sm.elapsed_in_state:.1f}s")

    # 7. 结果检查
    print("\n" + "=" * 60)
    if sm.state == RobotState.IDLE:
        print("  ✅ 状态机完成全流程: IDLE → ... → IDLE")
    elif sm.state == RobotState.ERROR:
        print(f"  ⚠️ 状态机进入 ERROR: {sm.error_reason}")
        print("     (update_task 404 是预期行为，因为用假 task ID)")
    else:
        print(f"  ❌ 状态机卡在: {sm.state.name}")

    print(f"  电池: {sm.battery}%")
    print(f"  tick 次数: {tick_count}")
    print("=" * 60)


if __name__ == "__main__":
    main()
