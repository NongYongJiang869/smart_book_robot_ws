'''
此代码为控制舵机的程序，主要通过串口发送指令给下位机Arduino执行动作，而Arduino已经集成代码
'''

from pyb import UART  #导入pyb模块中的UART、用于串口通信
import time #导入time模块，用于时间相关的功能，如延时和计时。


class Duoji():
    P0_INIT = 2200 #舵机的初始位置
    P2_INIT = 1500 #舵机的初始位置
    P3_INIT = 2200 #舵机的初始位置
    '''
    初始化串口
    '''
    def __init__(self):
        self.uart = UART(3,115200) #创建一个UART对象，使用第3个UART接口，设置波特率为115200。用于与其他设备进行串口通信。
        self.uart.init(115200, bits=8, parity=None, stop=1) #初始化UART接口，设置波特率为115200，数据位为8位，无奇偶校验，停止位为1位。确保串口通信的参数正确配置，以便与其他设备进行有效的通信。

        self.uart.read() #先清空一下缓冲区，以免之前的指令干扰到后续的通信。
        # 初始化 PID 控制器参数和状态
        # PID 用于对 Tx (左右) 和 Tz (远近) 进行闭环控制，输出为舵机位置增量
        self.pid_x = None
        self.pid_z = None
        self._last_time = None
        self._init_pid_controllers()
        # 用于过冲检测与步进记录
        self._prev_error_x = None
        self._prev_error_z = None
        self._last_delta_p0 = 0
        self._last_delta_p2 = 0
        self._last_delta_p3 = 0

    '''
    发送数据到下位机
    '''
    def send_str(self,cmd:str):
        self.uart.write(cmd)

    '''
    此为舵机的开始动作
    '''
    def run_duoji_kaidz(self):
        self.send_str("{G0003#000P2200T1000!#001P1500T1000!#002P1500T1000!#003P2200T1000!#004P1500T1000!#005P1000T1000!}")
        time.sleep_ms(1000)

    '''
    此为舵机的结束动作
    '''
    def run_duoji_guandz(self):
        self.send_str("#005P1700T800!")
        time.sleep_ms(800)
        self.send_str("{G0004#000P2200T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1500T1000!#005P1700T1000!}")
        time.sleep_ms(1000)
        self.send_str("{G0004#000P1500T1000!#001P1500T1000!#002P1500T1000!#003P2200T1000!#004P1500T1000!#005P1700T1000!}")
        time.sleep_ms(1000)
        self.send_str("{G0005#000P1500T1000!#001P2200T1000!#002P2400T1000!#003P2000T1000!#004P800T1000!#005P1700T1000!}")
        time.sleep_ms(1000)

    '''
    此为舵机的放书动作
    '''
    def run_duoji_fang(self):
        self.send_str("{G0009#000P1500T1000!#001P1300T1000!#002P1900T1000!#003P1900T1000!#004P800T1000!#005P1700T1000!}")
        time.sleep_ms(1000)
        self.send_str("#005P1000T800!")
        time.sleep_ms(800)
        self.send_str("{G0010#000P1500T1000!#001P2250T1000!#002P2310T1000!#003P2100T1000!#004P1500T1000!#005P1000T1000!}")
        time.sleep_ms(1000)

    '''
    根据输入的坐标信息，计算出各个舵机的角度，最后发送给下位机
    '''
    def run_duoji(self,x,y,z,rx,ry,rz):
        # 如果没有检测到数据，直接返回继续拍摄
        if x is None or z is None:
            return 1

        # 设定目标值（可调整）
        target_x = 3.0   # 画面左右居中目标值
        target_z = -4.0  # 合适抓取的距离目标值

        # 计算时间增量
        now = time.ticks_ms()
        if self._last_time is None:
            dt = 0.05
        else:
            dt = max(0.001, (time.ticks_diff(now, self._last_time) / 1000.0))
        self._last_time = now

        # 计算当前位置误差
        error_x = target_x - x
        error_z = target_z - z

        # 容差与粗调/细调逻辑
        tol_x = 0.8   # 允许 X 轴约 0.8 的误差即可抓取
        tol_z = 0.6   # 允许 Z 轴约 1.2 的误差即可抓取
        move_x = abs(error_x) >= tol_x
        move_z = abs(error_z) >= tol_z

        # 如果 X 轴偏移较大，则暂停 Z 前进，避免臂先前进导致偏下或丢失标签
        z_allowed_only_when_x_centered = abs(error_x) < tol_x
        if not z_allowed_only_when_x_centered:
            move_z = False
            try:
                self.pid_z._integ = 0
            except Exception:
                pass

        # 粗调阈值（远离目标时使用更大步进以加速）
        coarse_x = 1.1
        coarse_z = 0.8

        # 根据距离选择单步最大幅度与缩放因子（粗调/细调）
        if abs(error_x) > coarse_x:
            max_step_x = 80
            scale_x = 1.2
        else:
            max_step_x = 20
            scale_x = 0.4

        if abs(error_z) > coarse_z:
            max_step_z = 80
            scale_z = 1.0
        else:
            max_step_z = 25
            scale_z = 0.35

        delta_p0 = 0
        delta_p2 = 0
        delta_p3 = 0

        # 过冲检测：如果上次 Z 的误差存在且本次误差变大，说明上次移动朝错误方向，需反向微调并清除积分
        if self._prev_error_z is not None and abs(error_z) > abs(self._prev_error_z) + 0.02:
            inv_p2 = -int(self._last_delta_p2 * 0.69)
            inv_p3 = -int(self._last_delta_p3 * 0.45)
            inv_p2 = self._clamp(inv_p2, -max_step_z, max_step_z)
            inv_p3 = self._clamp(inv_p3, -max_step_z, max_step_z)
            self.P2_INIT = self._clamp(self.P2_INIT + inv_p2, 1400, 2300)
            self.P3_INIT = self._clamp(self.P3_INIT + inv_p3, 1400, 2300)
            try:
                self.pid_z._integ = 0
            except Exception:
                pass
            cmd = "{G0008#002P%dT400!#003P%dT400!}" % (self.P2_INIT, self.P3_INIT)
            self.send_str(cmd)
            time.sleep_ms(400)
            self._last_delta_p2 = inv_p2
            self._last_delta_p3 = inv_p3
            self._prev_error_z = error_z
            return 1

        # 过冲检测：如果上次 X 的误差存在且本次误差变大，也做反向微调并清除积分
        if self._prev_error_x is not None and abs(error_x) > abs(self._prev_error_x) + 0.02:
            inv_p0 = -int(self._last_delta_p0 * 0.5)
            inv_p0 = self._clamp(inv_p0, -max_step_x, max_step_x)
            self.P0_INIT = self._clamp(self.P0_INIT + inv_p0, 1500, 2300)
            try:
                self.pid_x._integ = 0
            except Exception:
                pass
            cmd = "#000P%dT400!" % (self.P0_INIT)
            self.send_str(cmd)
            time.sleep_ms(400)
            self._last_delta_p0 = inv_p0
            self._prev_error_x = error_x
            return 1

        # 正常 PID 计算与映射
        if move_x:
            ctrl_x = self.pid_x.compute(setpoint=target_x, measurement=x, dt=dt)
            delta_p0 = int(ctrl_x * scale_x)
            if delta_p0 > max_step_x:
                delta_p0 = max_step_x
            if delta_p0 < -max_step_x:
                delta_p0 = -max_step_x

        if move_z:
            ctrl_z = self.pid_z.compute(setpoint=target_z, measurement=z, dt=dt)
            delta_p2 = int(ctrl_z * scale_z)
            delta_p3 = -int(ctrl_z * scale_z)
            if delta_p2 > max_step_z:
                delta_p2 = max_step_z
            if delta_p2 < -max_step_z:
                delta_p2 = -max_step_z
            if delta_p3 > max_step_z:
                delta_p3 = max_step_z
            if delta_p3 < -max_step_z:
                delta_p3 = -max_step_z

        if delta_p0 != 0 or delta_p2 != 0 or delta_p3 != 0:
            self.P0_INIT = self._clamp(self.P0_INIT + delta_p0, 1500, 2300)
            self.P2_INIT = self._clamp(self.P2_INIT + delta_p2, 1400, 2300)
            self.P3_INIT = self._clamp(self.P3_INIT + delta_p3, 1400, 2300)
            cmd = "{G0008#000P%dT500!#002P%dT500!#003P%dT500!}" % (self.P0_INIT, self.P2_INIT, self.P3_INIT)
            self.send_str(cmd)
            time.sleep_ms(500)

        self._last_delta_p0 = delta_p0
        self._last_delta_p2 = delta_p2
        self._last_delta_p3 = delta_p3
        self._prev_error_x = error_x
        self._prev_error_z = error_z

        if abs(error_x) < tol_x and abs(error_z) < tol_z:
            self.send_str("#005P1700T800!")
            time.sleep_ms(800)
            return 0

        return 1
        # 发送数据到下位机

    def _clamp(self, v, lo, hi):
        if v < lo:
            return lo
        if v > hi:
            return hi
        return v

    def _init_pid_controllers(self):
        # 简单的 PID 实现用于 OpenMV（微控制器上运行，尽量保持精简）
        class PID:
            def __init__(self, kp, ki, kd, out_min=-1000, out_max=1000, integ_max=500):
                self.kp = kp
                self.ki = ki
                self.kd = kd
                self.out_min = out_min
                self.out_max = out_max
                self.integ_max = integ_max
                self._prev_error = 0.0
                self._integ = 0.0

            def compute(self, setpoint, measurement, dt):
                error = setpoint - measurement
                self._integ += error * dt
                # 防止积分风up
                if self._integ > self.integ_max:
                    self._integ = self.integ_max
                if self._integ < -self.integ_max:
                    self._integ = -self.integ_max

                deriv = 0.0
                if dt > 0:
                    deriv = (error - self._prev_error) / dt

                out = self.kp * error + self.ki * self._integ + self.kd * deriv
                # 限幅
                if out > self.out_max:
                    out = self.out_max
                if out < self.out_min:
                    out = self.out_min

                self._prev_error = error
                return out

        # 对 X（左右）使用较灵敏的增益，输出映射到 P0 的增量
        self.pid_x = PID(kp=5.0, ki=3.5, kd=2.1, out_min=-100, out_max=100, integ_max=100)
        # 对 Z（远近）使用较大范围调整，映射到 P2/P3
        self.pid_z = PID(kp=5.1, ki=3.1, kd=2.1, out_min=-100, out_max=100, integ_max=100)
        # 发送数据到下位机

    # 外部接口：设置 X 轴 PID
    def set_pid_x(self, kp, ki, kd):
        try:
            self.pid_x.kp = float(kp)
            self.pid_x.ki = float(ki)
            self.pid_x.kd = float(kd)
            return True
        except Exception:
            return False

    # 外部接口：设置 Z 轴 PID
    def set_pid_z(self, kp, ki, kd):
        try:
            self.pid_z.kp = float(kp)
            self.pid_z.ki = float(ki)
            self.pid_z.kd = float(kd)
            return True
        except Exception:
            return False

    # 外部接口：获取当前 PID 参数（x 和 z）
    def get_pid_status(self):
        try:
            return (self.pid_x.kp, self.pid_x.ki, self.pid_x.kd,
                    self.pid_z.kp, self.pid_z.ki, self.pid_z.kd)
        except Exception:
            return None
