#!/usr/bin/env python3
"""
全自动轮距标定脚本

用法:
  python3 tools/auto_calibrate.py

前置条件:
  - STM32 底盘已上电, 串口 /dev/ttyS1 已连接

流程:
  1. 启动 stm32_bridge 节点
  2. 开始标定 → 自动发送旋转指令 (正转+反转)
  3. 结束标定 → 自动更新 stm32_params.yaml
"""

import os
import sys
import time
import math
import signal
import subprocess
import yaml

# 路径
YAML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '../RDKX5/src/stm32_bridge/config/stm32_params.yaml')
WS_INSTALL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '../RDKX5/install/setup.bash')

# 所有子进程共享的 ROS2 source 命令
ROS2_ENV = f'source /opt/ros/humble/setup.bash && source {WS_INSTALL} && '

# 标定参数
ROTATE_SPEED = 0.5       # 旋转角速度 (rad/s)
ROTATE_TIME  = 12.0      # 每方向旋转时长 (s), 正反转各一次
                         # 0.5 rad/s × 12s ≈ 6 rad ≈ 344° × 2方向 ≈ 688°


def sh(cmd, timeout=None):
    """运行 bash 命令 (自动 source ROS2), 返回 CompletedProcess"""
    full = f"bash -c '{ROS2_ENV} {cmd}'"
    return subprocess.run(full, shell=True, capture_output=True,
                          text=True, timeout=timeout)


def ros2_service(path, srv_type, data, timeout=5):
    """调用 ROS2 服务"""
    cmd = f'ros2 service call {path} {srv_type} "{data}"'
    r = sh(cmd, timeout=timeout)
    if r.returncode != 0:
        print(f"  ✗ 服务调用失败: {r.stderr.strip()}")
        return None
    return r.stdout


def ros2_service_async(path, srv_type, data):
    """异步调用 ROS2 服务 (fire and forget)"""
    cmd = f'ros2 service call {path} {srv_type} "{data}"'
    return subprocess.Popen(
        f"bash -c '{ROS2_ENV} {cmd}'",
        shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def parse_calib_result(stdout):
    """从服务返回的 stdout 中提取 calibration 值"""
    result = {}
    for line in stdout.split('\n'):
        line = line.strip()
        if 'calibrated_wheel_base=' in line:
            try:
                result['wb'] = float(line.split('=')[1].strip())
            except:
                pass
        if 'correction_factor=' in line:
            try:
                result['factor'] = float(line.split('=')[1].strip())
            except:
                pass
        if 'success=' in line:
            result['success'] = 'True' in line or 'true' in line
        if 'message=' in line:
            result['msg'] = line.split('message=', 1)[1].strip().strip("'\"")
    return result


def main():
    print("=" * 60)
    print("  全自动轮距标定")
    print("=" * 60)
    print()

    # ── 0. 检查串口 ──
    if os.path.exists('/dev/ttyS1'):
        print("  ✓ 串口 /dev/ttyS1 已就绪")
    else:
        print("  ⚠ 未检测到 /dev/ttyS1")
        print("    请确认 STM32 底盘已上电且串口线已连接")
        print("    按 Enter 继续尝试...")
        input()

    # ── 1. 清理旧进程 ──
    print("[1/5] 清理旧进程...")
    subprocess.run('pkill -f stm32_bridge_node 2>/dev/null', shell=True)
    time.sleep(1.5)

    # ── 2. 启动 bridge 节点 ──
    print("[2/5] 启动 stm32_bridge...")
    bridge_proc = subprocess.Popen(
        ['bash', '-c',
         f'{ROS2_ENV} ros2 run stm32_bridge stm32_bridge_node'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("  等待节点就绪...", end=' ', flush=True)
    for i in range(20):
        time.sleep(1)
        r = sh('ros2 node list 2>/dev/null', timeout=3)
        if r and 'stm32_bridge' in (r.stdout or ''):
            print("OK")
            break
        print(".", end='', flush=True)
    else:
        print(" 超时!")
        print("  请确认: 1)底盘已上电 2)串口线已连 3)/dev/ttyS1 可访问")
        bridge_proc.terminate()
        sys.exit(1)

    # 等待话题和服务就绪
    print("  等待服务就绪...", end=' ', flush=True)
    for i in range(10):
        time.sleep(1)
        r = sh('ros2 service list 2>/dev/null', timeout=3)
        if r and '/calibrate_wheel_base' in (r.stdout or ''):
            print("OK")
            break
        print(".", end='', flush=True)
    else:
        print(" 超时!")
        bridge_proc.terminate()
        sys.exit(1)

    # ── 3. 开始标定 ──
    print("[3/5] 开始标定...")
    stdout = ros2_service('/calibrate_wheel_base',
                          'custom_interfaces/srv/CalibrateWheelBase',
                          '{start: true}')
    if stdout is None:
        bridge_proc.terminate()
        sys.exit(1)

    # ── 4. 旋转 ──
    print("[4/5] 自动旋转中...")

    # 发零速指令确保初始状态干净
    sh('ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
       '"{linear: {x: 0.0}, angular: {z: 0.0}}" 2>/dev/null', timeout=3)

    for direction, label in [('0.5', '顺时针'), ('-0.5', '逆时针')]:
        print(f"  → {label} {ROTATE_TIME}s @ {abs(float(direction))} rad/s")
        rot_proc = subprocess.Popen(
            ['bash', '-c',
             f'{ROS2_ENV} ros2 topic pub -r 20 /cmd_vel '
             f'geometry_msgs/msg/Twist '
             f'"{{linear: {{x: 0.0}}, angular: {{z: {direction}}}}}"'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(ROTATE_TIME)
        rot_proc.terminate()
        rot_proc.wait()
        # 刹车
        sh('ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
           '"{linear: {x: 0.0}, angular: {z: 0.0}}" 2>/dev/null', timeout=3)
        time.sleep(1)
    print("  旋转完成")

    # ── 5. 结束标定 ──
    print("[5/5] 结束标定, 计算结果...")
    stdout = ros2_service('/calibrate_wheel_base',
                          'custom_interfaces/srv/CalibrateWheelBase',
                          '{start: false}')
    if stdout is None:
        bridge_proc.terminate()
        sys.exit(1)

    result = parse_calib_result(stdout)
    print()
    print("=" * 60)
    if result.get('success'):
        wb = result['wb']
        print(f"  ✓ 标定成功!")
        print(f"  修正系数:           {result.get('factor', '?'):.4f}")
        print(f"  修正后 wheel_base:  {wb:.4f} m")
        print()

        # 更新 yaml
        try:
            with open(YAML_PATH, 'r') as f:
                config = yaml.safe_load(f)
            old_wb = config['stm32_bridge']['ros__parameters']['wheel_base']
            config['stm32_bridge']['ros__parameters']['wheel_base'] = round(wb, 4)
            with open(YAML_PATH, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
            print(f"  ✓ 已更新 stm32_params.yaml: {old_wb:.3f} → {wb:.4f}")
        except Exception as e:
            print(f"  ✗ 更新 yaml 失败: {e}")
            print(f"  请手动将 wheel_base 改为: {wb:.4f}")
    else:
        print(f"  ✗ 标定失败: {result.get('msg', '未知错误')}")
        print()
        print(f"  可能原因:")
        print(f"    1. 底盘没有上电或串口未连接")
        print(f"    2. 电机没有实际转动 (检查电池/接线)")
        print(f"    3. 编码器没有信号 (检查编码器接线)")
        print(f"  解决方法: 确认底盘正常后, 重新运行本脚本")
    print("=" * 60)

    # 清理
    bridge_proc.terminate()
    try:
        bridge_proc.wait(timeout=5)
    except:
        bridge_proc.kill()


if __name__ == '__main__':
    main()
