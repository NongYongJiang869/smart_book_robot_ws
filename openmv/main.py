'''
此代码为OpenMV控制舵机的主函数,将重复执行此函数

串口连接:
  UART1 (P1) ←→ RDK X5   — 接收控制指令、AprilTag 触发指令、机械臂透传指令
  UART3 (P4) ←→ 机械臂    — 透传 RDK X5 发来的原始机械臂协议指令，并回传响应

机械臂协议透传说明:
  RDK X5 发送以 $ / # / { / < 开头的指令时，OpenMV 不解析，
  直接转发到 UART3（机械臂），并将机械臂的响应回传 UART1（RDK X5）。
  支持的机械臂指令:
    $KMS:x,y,z,time!   — 逆运动学笛卡尔移动
    #xxxPyyyyTzzzz!    — 单舵机/全舵机 PWM 控制
    $DST! / $DST:N!    — 停止
    $RST!              — 软复位
    $QSTAT!            — 查询状态
    $QPWM!             — 查询 PWM
    $DGT:s-e,times!    — 执行动作组
    {Gxxxx#...!...}    — 动作组执行
    <Gxxxx#...!...>    — 动作组下载
'''
from pyb import UART  #导入pyb模块中的UART、用于串口通信
import time,pyb  #导入time模块，用于时间相关的功能，如延时和计时,以及pyb模块，用于访问OpenMV Cam的硬件功能。
import AprilTag,duoji  #导入aprilTag模块，用于AprilTag标记检测功能。以及duoji模块用与开始动作与结束动作

led =pyb.LED(3) #创建一个LED对象，控制OpenMV Cam上的第一个LED灯。为红色LED灯。

April = AprilTag.AprilTag() #创建一个AprilTag的对象，方便使用里面的函数
DuoJi = duoji.Duoji()


uart1 = UART(1,115200) #创建一个UART对象，使用第1个UART接口，设置波特率为115200。用于与其他设备进行串口通信。
uart1.init(115200, bits=8, parity=None, stop=1) #初始化UART接口，设置波特率为115200，数据位为8位，无奇偶校验，停止位为1位。确保串口通信的参数正确配置，以便与其他设备进行有效的通信。

# 复用 DuoJi 已初始化的 UART3，用于与机械臂通信
# DuoJi.__init__ 中已执行 UART(3, 115200).init(...)
arm_uart = DuoJi.uart

# 机械臂协议指令的起始字符
ARM_CMD_STARTS = (b'$', b'#', b'{', b'<')

#--至此完成串口初始化与配置，主要为了做一个舵机启动的的信号，舵机的启动将闪烁OpenMV的蓝灯

def LED():
    led.on() #打开LED灯
    time.sleep_ms(150) #延时150毫秒
    led.off() #关闭LED灯

def Tag(id):

    DuoJi.run_duoji_kaidz()
    while True:
        if April.run_kaishi(id) == 0:
            break
    DuoJi.run_duoji_guandz()
    uart1.write(b'0') #发送0到串口1，表示舵机已经完成动作，可以停止了。


def arm_passthrough(data):
    '''
    将 RDK X5 发来的机械臂原始指令透传到 UART3（机械臂），
    并等待机械臂响应，将响应回传到 UART1（RDK X5）。
    返回 True 表示已处理（是机械臂指令），False 表示不是机械臂指令。
    '''
    # 以 $ # { < 开头的才是机械臂协议指令
    if not data or data[0:1] not in ARM_CMD_STARTS:
        return False

    # 转发到机械臂
    arm_uart.write(data)

    # 等待机械臂响应（机械臂处理指令需要时间，典型响应延迟 30-200ms）
    # 分段等待，尽快把响应传回去
    timeout_ms = 300
    waited = 0
    while waited < timeout_ms:
        time.sleep_ms(10)
        waited += 10
        if arm_uart.any():
            resp = arm_uart.read()
            if resp:
                uart1.write(resp)
                # 继续读一下，有些指令会发多行响应
                time.sleep_ms(20)
                if arm_uart.any():
                    resp2 = arm_uart.read()
                    if resp2:
                        uart1.write(resp2)
                break

    return True


def forward_arm_response():
    '''
    透传机械臂的异步响应到 RDK X5。
    机械臂在执行完动作组或某些长操作后，可能会主动发送数据（如 @GroupDone!）。
    '''
    if arm_uart.any():
        resp = arm_uart.read()
        if resp:
            uart1.write(resp)


April.init() #初始化AprilTag

while True:
    # --- 1. 透传机械臂的异步响应 ---
    forward_arm_response()

    # --- 2. 处理 RDK X5 发来的数据 ---
    if uart1.any():
        data=uart1.read()  #读取收到的字节
        if not data:
            continue

        # --- 2a. 机械臂协议透传 ---
        # 如果数据以 $ # { < 开头，说明是发给机械臂的原始指令，直接透传
        if arm_passthrough(data):
            continue

        # --- 2b. OpenMV 本地指令处理 ---
        # 常用命令：'1'、'2' 触发抓取；'PX' 设置 X 轴 PID；'PZ' 设置 Z 轴 PID；'DBG' 查询当前 PID
        if b'1' in data:
            LED() #调用beep函数，发送蜂鸣器指令并闪烁LED灯，作为舵机启动的信号。
            Tag(0)
        elif b'2' in data:
            LED()
            Tag(1)
        elif b'3' in data:
            DuoJi.run_duoji_fang()
            uart1.write(b'6') #发送0到串口1，表示舵机已经完成动作，可以停止了。
        elif b'PX' in data:
            # 格式示例： PX:600,20,120
            try:
                payload = data.split(b'PX',1)[1].lstrip(b':=').strip()
                parts = payload.split(b',')
                if len(parts) >= 3:
                    kp = float(parts[0].decode())
                    ki = float(parts[1].decode())
                    kd = float(parts[2].decode())
                    if DuoJi.set_pid_x(kp,ki,kd):
                        uart1.write(b'OKPX')
                    else:
                        uart1.write(b'ERRPX')
                else:
                    uart1.write(b'ERRPX')
            except Exception:
                uart1.write(b'ERRPX')
        elif b'PZ' in data:
            # 格式示例： PZ:250,5,60
            try:
                payload = data.split(b'PZ',1)[1].lstrip(b':=').strip()
                parts = payload.split(b',')
                if len(parts) >= 3:
                    kp = float(parts[0].decode())
                    ki = float(parts[1].decode())
                    kd = float(parts[2].decode())
                    if DuoJi.set_pid_z(kp,ki,kd):
                        uart1.write(b'OKPZ')
                    else:
                        uart1.write(b'ERRPZ')
                else:
                    uart1.write(b'ERRPZ')
            except Exception:
                uart1.write(b'ERRPZ')
        elif b'DBG' in data:
            status = DuoJi.get_pid_status()
            if status:
                try:
                    s = 'PX:{:.1f},{:.1f},{:.1f};PZ:{:.1f},{:.1f},{:.1f}'.format(status[0],status[1],status[2],status[3],status[4],status[5])
                    uart1.write(s.encode())
                except Exception:
                    uart1.write(b'ERRDBG')
            else:
                uart1.write(b'ERRDBG')
