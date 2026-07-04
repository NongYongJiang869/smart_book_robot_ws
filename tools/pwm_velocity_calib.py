#!/usr/bin/env python3
"""
重车 PWM ←→ 速度 关系标定工具

用途:
  找出小车实际能动的最小 PWM / 最小速度，以及 PWM-速度对应关系。
  陀螺仪用作角速度参考（不受轮子打滑影响）。

原理:
  - 逐步递增 /cmd_vel 的 linear.x（测线速度）和 angular.z（测角速度）
  - 每个速度档位持续 2~3 秒，采集稳态响应
  - 线速度反馈: /odom (编码器)
  - 角速度反馈: /imu (陀螺仪 Z 轴，免疫侧滑)
  - 同时计算等效 PWM (基于 STM32 固件公式: pwm = v / 0.5 * 999)

用法:
  python3 tools/pwm_velocity_calib.py                          # 全量测试
  python3 tools/pwm_velocity_calib.py --linear-only            # 只测线速度
  python3 tools/pwm_velocity_calib.py --angular-only           # 只测角速度
  python3 tools/pwm_velocity_calib.py --csv result.csv         # 保存 CSV
  python3 tools/pwm_velocity_calib.py --max-linear 0.3         # 线速度上限

操作:
  小车放在地面上，脚本自动发指令，无需人工干预。
  测试期间请勿遥控小车。
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# ── QoS: 与 stm32_bridge 的 QOS_SENSOR 一致 ──
QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── 与 STM32 main.c 一致的参数 ──
MAX_PWM = 999
MAX_LINEAR_SPEED = 0.5      # m/s，对应 PWM=999
MAX_ANGULAR_SPEED = 2.0     # rad/s
WHEEL_BASE = 0.35           # m (与 STM32 固件一致)

# ANSI
CSI = '\033['
def sgr(*codes) -> str:
    return f"{CSI}{';'.join(map(str, codes))}m"

RST = sgr(0); BOLD = sgr(1); DIM = sgr(2)
GREEN = sgr(32); YELLOW = sgr(33); RED = sgr(31)
CYAN = sgr(36); MAG = sgr(35)
NL = '\n'


def velocity_to_pwm(linear_x: float, angular_z: float) -> Tuple[float, float]:
    """cmd_vel → 左右轮等效 PWM (与 STM32 vel_to_pwm 逻辑一致)"""
    v_left  = linear_x - angular_z * WHEEL_BASE / 2.0
    v_right = linear_x + angular_z * WHEEL_BASE / 2.0
    v_left  = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, v_left))
    v_right = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, v_right))
    pwm_left  = v_left  / MAX_LINEAR_SPEED * MAX_PWM
    pwm_right = v_right / MAX_LINEAR_SPEED * MAX_PWM
    return (pwm_left, pwm_right)


class CalibNode(Node):
    """标定节点: 发 cmd_vel，收 odom + imu"""

    def __init__(self):
        super().__init__('pwm_velocity_calib')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 最新数据
        self.odom_vx: float = 0.0
        self.odom_vth: float = 0.0
        self.gyro_z: float = 0.0       # rad/s (从 /imu)
        self._odom_ts = None
        self._imu_ts = None

        # 订阅
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, QOS_SENSOR)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_cb, QOS_SENSOR)

    def _odom_cb(self, msg: Odometry):
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_vth = msg.twist.twist.angular.z
        self._odom_ts = time.time()

    def _imu_cb(self, msg: Imu):
        self.gyro_z = msg.angular_velocity.z   # rad/s
        self._imu_ts = time.time()

    def send_vel(self, linear_x: float, angular_z: float):
        """发送速度指令"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def stop(self):
        """发送零速"""
        self.send_vel(0.0, 0.0)

    def wait_for_data(self, timeout: float = 1.0) -> bool:
        """等待 odom 和 imu 数据到达"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._odom_ts and self._imu_ts:
                return True
        return False


def collect_sample(node: CalibNode, linear_x: float, angular_z: float,
                   settle_time: float = 2.0, sample_time: float = 1.0,
                   samples: int = 10) -> dict:
    """
    发送速度指令，等待稳定后采集数据。

    Args:
        settle_time: 等待电机稳定的时间 (秒)
        sample_time: 采集窗口 (秒)
        samples:     采集次数

    Returns:
        dict with averaged results
    """
    node.send_vel(linear_x, angular_z)

    # 等待稳定
    t0 = time.time()
    while time.time() - t0 < settle_time:
        rclpy.spin_once(node, timeout_sec=0.05)

    # 采集数据
    vx_vals, vth_vals, gyro_vals = [], [], []
    t_start = time.time()
    while time.time() - t_start < sample_time:
        rclpy.spin_once(node, timeout_sec=0.05)
        vx_vals.append(node.odom_vx)
        vth_vals.append(node.odom_vth)
        gyro_vals.append(node.gyro_z)

    if not vx_vals:
        return None

    pwm_l, pwm_r = velocity_to_pwm(linear_x, angular_z)

    return {
        'linear_x_cmd':  linear_x,
        'angular_z_cmd': angular_z,
        'pwm_left':      pwm_l,
        'pwm_right':     pwm_r,
        'pwm_max_abs':   max(abs(pwm_l), abs(pwm_r)),
        'odom_vx_mean':  sum(vx_vals) / len(vx_vals),
        'odom_vx_std':   (sum((v - sum(vx_vals)/len(vx_vals))**2
                             for v in vx_vals) / len(vx_vals)) ** 0.5,
        'odom_vth_mean': sum(vth_vals) / len(vth_vals),
        'gyro_z_mean':   sum(gyro_vals) / len(gyro_vals),
        'gyro_z_std':    (sum((g - sum(gyro_vals)/len(gyro_vals))**2
                             for g in gyro_vals) / len(gyro_vals)) ** 0.5,
        'samples':       len(vx_vals),
    }


def build_linear_speeds(min_v: float = 0.01, max_v: float = 0.2,
                         step_small: float = 0.01, step_big: float = 0.02,
                         threshold: float = 0.06) -> List[float]:
    """生成线速度测试序列: 低速段细步长，高速段粗步长"""
    speeds = []
    v = min_v
    while v <= threshold + 0.0001:
        speeds.append(round(v, 3))
        v += step_small
    while v <= max_v + 0.0001:
        speeds.append(round(v, 3))
        v += step_big
    return speeds


def build_angular_speeds(min_w: float = 0.05, max_w: float = 1.0,
                          step_small: float = 0.05, step_big: float = 0.1,
                          threshold: float = 0.3) -> List[float]:
    """生成角速度测试序列"""
    speeds = []
    w = min_w
    while w <= threshold + 0.0001:
        speeds.append(round(w, 3))
        w += step_small
    while w <= max_w + 0.0001:
        speeds.append(round(w, 3))
        w += step_big
    return speeds


def find_threshold(results: List[dict], key_cmd: str, key_actual: str,
                   dead_ratio: float = 0.05) -> Optional[dict]:
    """
    找出「小车开始动」的最小指令值。
    判断标准: 实际速度 / 指令速度 < dead_ratio 视为未动。
    """
    for r in results:
        cmd = abs(r[key_cmd])
        actual = abs(r[key_actual])
        if cmd < 0.0001:
            continue
        ratio = actual / cmd
        if ratio > dead_ratio:
            return r
    return None


def print_separator(ch: str = '─', width: int = 80):
    print(f"{DIM}{ch * width}{RST}")


def print_line(cmd_pwm, cmd_vel, actual_enc, actual_gyro,
               moving: bool, unit: str = 'm/s'):
    """打印一行测试结果"""
    status = f"{GREEN}✓ 动{RST}" if moving else f"{RED}✗ 静{RST}"

    if unit == 'rad/s':
        print(f"  PWM={cmd_pwm:6.0f}  "
              f"指令={cmd_vel:+7.3f} rad/s  "
              f"编码器={actual_enc:+8.4f} rad/s  "
              f"陀螺仪={actual_gyro:+8.4f} rad/s  "
              f"{status}")
    else:
        print(f"  PWM={cmd_pwm:6.0f}  "
              f"指令={cmd_vel:+7.3f} m/s  "
              f"编码器={actual_enc:+8.4f} m/s  "
              f"陀螺仪={actual_gyro:+8.4f} rad/s  "
              f"{status}")


def print_summary(linear_results: List[dict], angular_results: List[dict]):
    """打印汇总报告"""
    print(f"{NL}{BOLD}{CYAN}{'═'*80}{RST}")
    print(f"{BOLD}{CYAN}  标定汇总{RST}")
    print(f"{BOLD}{CYAN}{'═'*80}{RST}{NL}")

    # ── 线速度 ──
    if linear_results:
        print(f"{BOLD}【线速度 — 最小能动阈值】{RST}")
        print_separator()
        th = find_threshold(linear_results, 'linear_x_cmd', 'odom_vx_mean')
        if th:
            pwm = max(abs(th['pwm_left']), abs(th['pwm_right']))
            print(f"  最小线速度指令: {th['linear_x_cmd']:.3f} m/s")
            print(f"  等效 PWM:        {pwm:.0f} / {MAX_PWM} ({pwm/MAX_PWM*100:.1f}%)")
            print(f"  实际速度:        {th['odom_vx_mean']:.4f} m/s")
        else:
            print(f"  {RED}未找到有效阈值 (所有指令都推不动车?){RST}")
        print()

        # 有效数据点 — PWM-速度曲线
        print(f"{BOLD}【线速度 — PWM→实际速度 映射】{RST}")
        print_separator()
        effective = [r for r in linear_results
                     if abs(r['odom_vx_mean']) > 0.001]
        if effective:
            print(f"  {'PWM':>6s}  {'指令 m/s':>9s}  {'实际 m/s':>9s}  {'比率':>7s}")
            print(f"  {'─'*6}  {'─'*9}  {'─'*9}  {'─'*7}")
            for r in effective:
                pwm = max(abs(r['pwm_left']), abs(r['pwm_right']))
                ratio = r['odom_vx_mean'] / r['linear_x_cmd'] if r['linear_x_cmd'] else 0
                print(f"  {pwm:6.0f}  {r['linear_x_cmd']:+9.3f}  "
                      f"{r['odom_vx_mean']:+9.4f}  {ratio:+7.3f}")
        else:
            print(f"  {RED}无有效数据点{RST}")
        print()

    # ── 角速度 ──
    if angular_results:
        print(f"{BOLD}【角速度 — 最小能动阈值】{RST}")
        print_separator()
        th = find_threshold(angular_results, 'angular_z_cmd', 'gyro_z_mean')
        if th:
            pwm = max(abs(th['pwm_left']), abs(th['pwm_right']))
            print(f"  最小角速度指令: {th['angular_z_cmd']:.3f} rad/s")
            print(f"  等效 PWM:        {pwm:.0f} / {MAX_PWM} ({pwm/MAX_PWM*100:.1f}%)")
            print(f"  陀螺仪反馈:      {th['gyro_z_mean']:.4f} rad/s")
        else:
            print(f"  {RED}未找到有效阈值 (所有指令都转不动车?){RST}")
        print()

        # 有效数据点
        print(f"{BOLD}【角速度 — PWM→陀螺仪反馈 映射】{RST}")
        print_separator()
        effective = [r for r in angular_results
                     if abs(r['gyro_z_mean']) > 0.005]
        if effective:
            print(f"  {'PWM':>6s}  {'指令 rad/s':>10s}  {'陀螺 rad/s':>10s}  "
                  f"{'编码 rad/s':>10s}  {'陀/指':>7s}")
            print(f"  {'─'*6}  {'─'*10}  {'─'*10}  {'─'*10}  {'─'*7}")
            for r in effective:
                pwm = max(abs(r['pwm_left']), abs(r['pwm_right']))
                ratio = r['gyro_z_mean'] / r['angular_z_cmd'] if r['angular_z_cmd'] else 0
                print(f"  {pwm:6.0f}  {r['angular_z_cmd']:+10.4f}  "
                      f"{r['gyro_z_mean']:+10.4f}  "
                      f"{r['odom_vth_mean']:+10.4f}  {ratio:+7.3f}")
        else:
            print(f"  {RED}无有效数据点{RST}")
        print()

    # ── 建议 ──
    print(f"{BOLD}{YELLOW}【标定建议】{RST}")
    print_separator()

    # 线速度阈值建议
    if linear_results:
        th_l = find_threshold(linear_results, 'linear_x_cmd', 'odom_vx_mean')
        if th_l:
            pwm_l = max(abs(th_l['pwm_left']), abs(th_l['pwm_right']))
            # 建议: 阈值 * 1.5 作为最小速度 (留余量)
            min_v = th_l['linear_x_cmd'] * 1.5
            print(f"  线速度:")
            print(f"    死区阈值: ~{th_l['linear_x_cmd']:.3f} m/s (PWM={pwm_l:.0f})")
            print(f"    建议 min_vel_x: {min_v:.3f} m/s (Nav2 参数)")
            print(f"    建议 STM32 MIN_EFFECTIVE_PWM: {int(pwm_l * 1.3)} (固件参数)")

    if angular_results:
        th_a = find_threshold(angular_results, 'angular_z_cmd', 'gyro_z_mean')
        if th_a:
            pwm_a = max(abs(th_a['pwm_left']), abs(th_a['pwm_right']))
            print(f"  角速度:")
            print(f"    死区阈值: ~{th_a['angular_z_cmd']:.3f} rad/s (PWM={pwm_a:.0f})")
            print(f"    建议 min_speed_theta: {th_a['angular_z_cmd'] * 1.5:.3f} rad/s (Nav2)")

    print()
    print(f"  {DIM}将建议值更新到:{RST}")
    print(f"  {DIM}  Nav2:  RDKX5/src/stm32_bridge/config/nav2_params.yaml{RST}")
    print(f"  {DIM}  STM32: chassis/src/main.c (MIN_EFFECTIVE_PWM 宏){RST}")


def _find_workspace() -> str:
    """查找 RDKX5 工作空间路径"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws = os.path.join(script_dir, '..', 'RDKX5')
    if os.path.isdir(ws):
        return os.path.abspath(ws)
    # 备选: 从环境变量或常见位置查找
    for path in ['/root/smart_book_robot_ws/RDKX5',
                 os.path.expanduser('~/smart_book_robot_ws/RDKX5')]:
        if os.path.isdir(path):
            return path
    return None


def _launch_chassis() -> Optional[subprocess.Popen]:
    """后台启动 stm32_bridge 节点"""
    ws = _find_workspace()
    if not ws:
        print(f"{RED}找不到 RDKX5 工作空间{RST}")
        return None

    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {ws}/install/setup.bash && "
        f"ros2 run stm32_bridge stm32_bridge_node --ros-args "
        f"-p serial_port:=/dev/ttyS1 "
        f"-p publish_odom_tf:=false "
    )
    print(f"  启动 stm32_bridge...")
    proc = subprocess.Popen(
        ['bash', '-c', cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return proc


def main():
    ap = argparse.ArgumentParser(description="重车 PWM-速度 标定工具")
    ap.add_argument('--linear-only', action='store_true', help='只测线速度')
    ap.add_argument('--angular-only', action='store_true', help='只测角速度')
    ap.add_argument('--max-linear', type=float, default=0.2,
                    help='线速度最大值 m/s (默认 0.2)')
    ap.add_argument('--max-angular', type=float, default=1.0,
                    help='角速度最大值 rad/s (默认 1.0)')
    ap.add_argument('--settle', type=float, default=2.0,
                    help='每个速度档稳定时间 秒 (默认 2.0)')
    ap.add_argument('--csv', default=None, help='CSV 输出文件')
    ap.add_argument('--no-auto-start', action='store_true',
                    help='不自动启动底盘 (需要手动先启动 stm32_bridge)')
    args = ap.parse_args()

    do_linear = not args.angular_only
    do_angular = not args.linear_only

    # ── 启动 ROS2 ──
    rclpy.init()
    node = CalibNode()
    chassis_proc = None

    print(f"{BOLD}{CYAN}{'═'*80}{RST}")
    print(f"{BOLD}{CYAN}  重车 PWM ←→ 速度 关系标定{RST}")
    print(f"{BOLD}{CYAN}{'═'*80}{RST}{NL}")
    print(f"  参数: MAX_PWM={MAX_PWM}, MAX_LINEAR={MAX_LINEAR_SPEED}m/s, "
          f"MAX_ANGULAR={MAX_ANGULAR_SPEED}rad/s")
    print(f"  PWM公式: pwm = (speed / {MAX_LINEAR_SPEED}) * {MAX_PWM}")
    print(f"  角速度: 陀螺仪参考 (免疫侧滑), 编码器辅助对比")
    print(f"{NL}  {YELLOW}⚠ 请确保小车在地面上，测试期间勿遥控！{RST}{NL}")

    # ── 等待数据 (自动启动底盘如需要) ──
    print("  检查 /odom 和 /imu 数据...", end=' ', flush=True)
    has_data = node.wait_for_data(timeout=2.0)

    if not has_data and not args.no_auto_start:
        print(f"{YELLOW}未检测到{RST}")
        print(f"  自动启动 stm32_bridge...")
        chassis_proc = _launch_chassis()
        if chassis_proc:
            print(f"  等待节点就绪...", end=' ', flush=True)
            # 等待最多 10 秒
            for _ in range(50):
                time.sleep(0.2)
                if node.wait_for_data(timeout=0.5):
                    has_data = True
                    break
            if has_data:
                print(f"{GREEN}OK{RST}{NL}")
            else:
                print(f"{RED}超时! stm32_bridge 启动失败{RST}")
        else:
            print(f"{RED}无法自动启动 stm32_bridge{RST}")
    elif has_data:
        print(f"{GREEN}OK{RST}{NL}")
    else:
        print(f"{RED}超时! 请手动启动 stm32_bridge{RST}")

    if not has_data:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # ── CSV ──
    csv_file = None
    csv_w = None
    if args.csv:
        csv_file = open(args.csv, 'w', newline='')
        csv_w = csv.writer(csv_file)
        csv_w.writerow(['test_type', 'linear_x_cmd', 'angular_z_cmd',
                        'pwm_left', 'pwm_right', 'pwm_max_abs',
                        'odom_vx_mean', 'odom_vx_std',
                        'odom_vth_mean',
                        'gyro_z_mean', 'gyro_z_std', 'samples'])

    # ── 清零 ──
    print("  先发零速确保停止...", end=' ', flush=True)
    node.stop()
    time.sleep(0.5)
    print("OK")
    print()

    linear_results = []
    angular_results = []

    def cleanup():
        node.stop()
        if csv_file:
            csv_file.close()
        if chassis_proc:
            chassis_proc.terminate()
            try:
                chassis_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                chassis_proc.kill()

    def on_signal(signum, frame):
        print(f"{NL}{YELLOW}收到中断信号，正在停车...{RST}")
        cleanup()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        # ════════════════════════════════════════════════════════════
        # 线速度测试
        # ════════════════════════════════════════════════════════════
        if do_linear:
            print(f"{BOLD}═══ 线速度测试 (angular_z = 0) ═══{RST}")
            print_separator()
            print(f"  {'PWM':>6s}  {'指令 m/s':>9s}  {'实际 m/s':>9s}  "
                  f"{'陀螺 rad/s':>10s}  {'状态'}")
            print_separator()

            speeds = build_linear_speeds(max_v=args.max_linear)
            for v in speeds:
                r = collect_sample(node, v, 0.0, settle_time=args.settle)
                if r is None:
                    continue
                linear_results.append(r)

                pwm = max(abs(r['pwm_left']), abs(r['pwm_right']))
                moving = abs(r['odom_vx_mean']) > 0.002
                print_line(pwm, v, r['odom_vx_mean'], r['gyro_z_mean'],
                          moving, unit='m/s')

                if csv_w:
                    csv_w.writerow(['linear', r['linear_x_cmd'], r['angular_z_cmd'],
                                    r['pwm_left'], r['pwm_right'], r['pwm_max_abs'],
                                    r['odom_vx_mean'], r['odom_vx_std'],
                                    r['odom_vth_mean'],
                                    r['gyro_z_mean'], r['gyro_z_std'], r['samples']])

            node.stop()
            time.sleep(0.5)
            print()

        # ════════════════════════════════════════════════════════════
        # 角速度测试
        # ════════════════════════════════════════════════════════════
        if do_angular:
            print(f"{BOLD}═══ 角速度测试 (linear_x = 0) ═══{RST}")
            print_separator()
            print(f"  {'PWM':>6s}  {'指令 rad/s':>10s}  "
                  f"{'陀螺 rad/s':>10s}  {'编码 rad/s':>10s}  {'状态'}")
            print_separator()

            speeds = build_angular_speeds(max_w=args.max_angular)
            for w in speeds:
                r = collect_sample(node, 0.0, w, settle_time=args.settle)
                if r is None:
                    continue
                angular_results.append(r)

                pwm = max(abs(r['pwm_left']), abs(r['pwm_right']))
                moving = abs(r['gyro_z_mean']) > 0.01   # 用陀螺判断是否真转了
                print_line(pwm, w, r['odom_vth_mean'], r['gyro_z_mean'],
                          moving, unit='rad/s')

                if csv_w:
                    csv_w.writerow(['angular', r['linear_x_cmd'], r['angular_z_cmd'],
                                    r['pwm_left'], r['pwm_right'], r['pwm_max_abs'],
                                    r['odom_vx_mean'], r['odom_vx_std'],
                                    r['odom_vth_mean'],
                                    r['gyro_z_mean'], r['gyro_z_std'], r['samples']])

            node.stop()
            time.sleep(0.5)
            print()

        # ════════════════════════════════════════════════════════════
        # 汇总
        # ════════════════════════════════════════════════════════════
        print_summary(linear_results, angular_results)

    finally:
        cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
