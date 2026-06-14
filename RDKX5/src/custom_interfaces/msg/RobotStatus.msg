# 机器人整体状态 (book_search_master 发布)
std_msgs/Header header

# 当前状态
uint8 state
uint8 IDLE=0
uint8 NAVIGATING=1
uint8 SCANNING=2
uint8 APPROACHING=3
uint8 PICKING=4
uint8 RETURNING=5
uint8 ERROR=6

# 底盘
float32 linear_velocity     # 当前线速度 (m/s)
float32 angular_velocity    # 当前角速度 (rad/s)

# 当前任务
string  current_task        # 正在找的书名, 空闲时为空

# 错误信息
string  error_message       # 错误描述, 无错误时为空
