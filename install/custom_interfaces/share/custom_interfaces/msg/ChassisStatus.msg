# 底盘底层状态 (stm32_bridge 发布, 用于诊断和监控)
std_msgs/Header header

# 4个电机使能状态 (位0~3, 1=使能)
uint8 motor_enabled

# 传感器状态
bool emergency_stop      # 急停是否触发
bool collision_front     # 前碰撞传感器
bool collision_rear      # 后碰撞传感器

# 错误码 (0=正常)
uint16 error_code
uint16 ERR_NONE=0
uint16 ERR_ESTOP=1
uint16 ERR_FRONT_COLLISION=2
uint16 ERR_REAR_COLLISION=4
uint16 ERR_COMM_TIMEOUT=256
uint16 ERR_IMU_FAULT=1024

# 通信质量
float32 cmd_latency_ms   # 最近一次速度指令的往返延迟
uint16  lost_frames      # 近1秒内丢帧数量
