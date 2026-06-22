#!/bin/bash
# 全自动轮距标定
set +e

source /opt/ros/humble/setup.bash
source /root/smart_book_robot_ws/RDKX5/install/setup.bash

echo "============================================================"
echo "  全自动轮距标定"
echo "============================================================"
echo ""

# 1. 清理
echo "[1/5] 清理旧进程..."
pkill -9 -f stm32_bridge_node 2>/dev/null
sleep 2

# 2. 启动 bridge (静默输出)
echo "[2/5] 启动 stm32_bridge..."
ros2 run stm32_bridge stm32_bridge_node >/dev/null 2>&1 &
BRIDGE_PID=$!
sleep 4

echo -n "  等待服务就绪..."
for i in $(seq 1 20); do
    if ros2 service list 2>/dev/null | grep -q calibrate_wheel_base; then
        echo " OK"
        break
    fi
    sleep 1
    echo -n "."
done

# 3. 开始标定
echo "[3/5] 开始标定..."
ros2 service call /calibrate_wheel_base custom_interfaces/srv/CalibrateWheelBase "{start: true}"
echo ""

# 4. 旋转 (2.0 rad/s → 70% PWM, 重车需要)
echo "[4/5] 自动旋转..."
echo "  → 顺时针 15s @ 2.0 rad/s"
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 2.0}}" >/dev/null 2>&1 &
ROT_PID=$!
sleep 15
kill $ROT_PID 2>/dev/null
wait $ROT_PID 2>/dev/null
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
sleep 2

echo "  → 逆时针 15s @ 2.0 rad/s"
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: -2.0}}" >/dev/null 2>&1 &
ROT_PID=$!
sleep 15
kill $ROT_PID 2>/dev/null
wait $ROT_PID 2>/dev/null
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
sleep 1
echo "  旋转完成"

# 5. 结束标定
echo "[5/5] 结束标定..."
RESULT=$(ros2 service call /calibrate_wheel_base \
    custom_interfaces/srv/CalibrateWheelBase "{start: false}" 2>&1)
echo "$RESULT"
echo ""

# 解析
SUCCESS=$(echo "$RESULT" | grep -oP 'success=\K\w+')
WB=$(echo "$RESULT" | grep -oP 'calibrated_wheel_base=\K[0-9.]+')
FACTOR=$(echo "$RESULT" | grep -oP 'correction_factor=\K[0-9.]+')

echo "============================================================"
if [ "$SUCCESS" = "True" ] || [ "$SUCCESS" = "true" ]; then
    echo "  ✓ 标定成功!"
    echo "  修正系数:           $FACTOR"
    echo "  修正后 wheel_base:  $WB m"
    echo ""

    YAML="/root/smart_book_robot_ws/RDKX5/src/stm32_bridge/config/stm32_params.yaml"
    python3 -c "
import yaml
with open('$YAML') as f:
    c = yaml.safe_load(f)
old = c['stm32_bridge']['ros__parameters']['wheel_base']
c['stm32_bridge']['ros__parameters']['wheel_base'] = round($WB, 4)
with open('$YAML', 'w') as f:
    yaml.dump(c, f, default_flow_style=False, allow_unicode=True)
print(f'  ✓ 已更新 stm32_params.yaml: {old:.3f} → {$WB:.4f}')
"
else
    echo "  ✗ 标定失败"
    echo "  请确认: 1)底盘上电 2)串口连接 3)电机编码器正常"
fi
echo "============================================================"

kill $BRIDGE_PID 2>/dev/null
wait $BRIDGE_PID 2>/dev/null
