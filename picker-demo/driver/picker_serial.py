"""
Picker Robot V2 Python Driver
基于 AR4 v5.1 串口协议，从 AR4.py 源码逆向

串口协议概要:
  - 波特率: 115200
  - 主控串口 (Teensy 4.1): 运动指令
  - 扩展板串口 (Arduino Nano): 舵机/IO 指令

指令格式:
  - RJ: 关节空间运动 (Run Joints)
  - MJ: 笛卡尔空间运动 (Move Joints - IK on Teensy)
  - ML: 笛卡尔空间直线运动 (Move Linear)
  - LJ: Live Jog
  - RP: 请求当前位置 (Request Position)
  - LL: 校准指令
  - SV: 舵机控制 (发到 Nano)
  - ON/OF: 数字输出开关 (发到 Nano)

响应格式:
  成功: A<j1>B<j2>C<j3>D<j4>E<j5>F<j6>G<x>H<y>I<z>J<rz>K<ry>L<rx>M<speedVio>N<debug>O<flag>P<j7>Q<j8>R<j9>
  错误: E + L<限位标志> 或 E + 其他错误码
"""

import serial
import time
import logging
from dataclasses import dataclass, field
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


@dataclass
class JointState:
    """6轴关节角度 (度)"""
    j1: float = 0.0
    j2: float = 0.0
    j3: float = 0.0
    j4: float = 0.0
    j5: float = 0.0
    j6: float = 0.0

    def as_tuple(self) -> Tuple[float, ...]:
        return (self.j1, self.j2, self.j3, self.j4, self.j5, self.j6)

    def __str__(self):
        return f"J[{self.j1:.2f}, {self.j2:.2f}, {self.j3:.2f}, {self.j4:.2f}, {self.j5:.2f}, {self.j6:.2f}]"


@dataclass
class CartesianPose:
    """笛卡尔空间位姿"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

    def __str__(self):
        return f"P[{self.x:.2f}, {self.y:.2f}, {self.z:.2f}, {self.rx:.2f}, {self.ry:.2f}, {self.rz:.2f}]"


@dataclass
class RobotPosition:
    """机器人完整状态"""
    joints: JointState = field(default_factory=JointState)
    pose: CartesianPose = field(default_factory=CartesianPose)
    j7: float = 0.0  # 外部轴
    j8: float = 0.0
    j9: float = 0.0
    speed_violation: str = ""
    debug: str = ""
    flag: str = ""


@dataclass
class MotionParams:
    """运动参数"""
    speed: float = 25.0       # 速度 (百分比 0-100)
    accel: float = 20.0       # 加速度
    decel: float = 20.0       # 减速度
    ramp: float = 100.0       # 加速曲线 (0-100, 100=梯形, 越小越S型)
    speed_type: str = "percent"  # "percent" | "seconds" | "mm_per_sec"
    rounding: float = 0.0     # 拐角圆滑 (仅 ML)


class PickerError(Exception):
    """Picker 通信/执行错误"""
    pass


class PickerAxisLimitError(PickerError):
    """关节超限错误"""
    def __init__(self, limits: dict):
        self.limits = limits
        msg = "Axis limit: " + ", ".join(f"J{k}" for k, v in limits.items() if v)
        super().__init__(msg)


class PickerDriver:
    """
    Picker Robot V2 驱动

    用法:
        picker = PickerDriver("/dev/tty.usbmodem...", "/dev/tty.usbmodem...")
        picker.connect()
        pos = picker.request_position()
        picker.move_joints(0, 0, 0, 0, 0, 0)
        picker.move_linear(200, 0, 300, 0, 0, 0)
        picker.gripper_open()
        picker.disconnect()
    """

    # Picker V2 的关节限位 (从 DH 参数截图)
    JOINT_LIMITS = {
        1: (-163, 163),
        2: (-60, 90),
        3: (-90, 85),
        4: (-173, 173),
        5: (-108, 108),
        6: (-180, 180),
    }

    def __init__(
        self,
        main_port: str,
        nano_port: Optional[str] = None,
        baudrate: int = 115200,
        timeout: float = 5.0,
    ):
        """
        Args:
            main_port: Teensy 4.1 主控串口
            nano_port: Arduino Nano 扩展板串口 (舵机/IO)
            baudrate: 波特率，默认 115200
            timeout: 串口读取超时 (秒)
        """
        self.main_port = main_port
        self.nano_port = nano_port
        self.baudrate = baudrate
        self.timeout = timeout

        self._ser: Optional[serial.Serial] = None
        self._ser_nano: Optional[serial.Serial] = None
        self._position = RobotPosition()
        self._motion_params = MotionParams()
        self._connected = False
        self._loop_mode = "000000"  # 6轴闭环模式

    # ─── 连接管理 ────────────────────────────────────────────

    def connect(self):
        """连接 Picker"""
        try:
            self._ser = serial.Serial(
                self.main_port, self.baudrate, timeout=self.timeout
            )
            logger.info(f"Main port connected: {self.main_port}")

            if self.nano_port:
                self._ser_nano = serial.Serial(
                    self.nano_port, self.baudrate, timeout=self.timeout
                )
                logger.info(f"Nano port connected: {self.nano_port}")

            time.sleep(2)  # 等待 Arduino 重启
            self._connected = True

            # 请求初始位置
            self.request_position()
            logger.info(f"Picker connected. Position: {self._position.joints}")

        except serial.SerialException as e:
            raise PickerError(f"Connection failed: {e}")

    def disconnect(self):
        """断开连接"""
        if self._ser and self._ser.is_open:
            self._ser.close()
        if self._ser_nano and self._ser_nano.is_open:
            self._ser_nano.close()
        self._connected = False
        logger.info("Picker disconnected")

    @property
    def is_connected(self) -> bool:
        return self._connected and self._ser is not None and self._ser.is_open

    # ─── 运动参数 ────────────────────────────────────────────

    def set_speed(self, speed: float):
        """设置速度百分比 (0-100)"""
        self._motion_params.speed = max(0, min(100, speed))

    def set_accel(self, accel: float):
        """设置加速度 (0-100)"""
        self._motion_params.accel = max(0, min(100, accel))

    def set_decel(self, decel: float):
        """设置减速度 (0-100)"""
        self._motion_params.decel = max(0, min(100, decel))

    def set_ramp(self, ramp: float):
        """设置加速曲线 (0-100)"""
        self._motion_params.ramp = max(0, min(100, ramp))

    # ─── 位置查询 ────────────────────────────────────────────

    def request_position(self) -> RobotPosition:
        """请求当前位置"""
        response = self._send_main("RP\n")
        self._parse_position(response)
        return self._position

    @property
    def position(self) -> RobotPosition:
        """最近一次的位置 (不发送请求)"""
        return self._position

    @property
    def joints(self) -> JointState:
        return self._position.joints

    @property
    def pose(self) -> CartesianPose:
        return self._position.pose

    # ─── 关节空间运动 ────────────────────────────────────────

    def move_joints(
        self,
        j1: float, j2: float, j3: float,
        j4: float, j5: float, j6: float,
        j7: float = 0, j8: float = 0, j9: float = 0,
        speed: Optional[float] = None,
        wait: bool = True,
    ) -> RobotPosition:
        """
        关节空间运动 (RJ 指令)

        Args:
            j1-j6: 目标关节角度 (度)
            j7-j9: 外部轴位置
            speed: 临时速度 (不改变默认值)
            wait: 是否等待运动完成

        Returns:
            运动完成后的位置
        """
        self._check_joint_limits(j1, j2, j3, j4, j5, j6)
        params = self._build_speed_params(speed)
        wc = self._calc_wrist_config(j5)

        cmd = (
            f"RJ"
            f"A{j1}B{j2}C{j3}D{j4}E{j5}F{j6}"
            f"J7{j7}J8{j8}J9{j9}"
            f"{params}W{wc}Lm{self._loop_mode}\n"
        )

        response = self._send_main(cmd)
        self._handle_response(response)
        return self._position

    def move_joint_relative(
        self, joint: int, delta: float, speed: Optional[float] = None
    ) -> RobotPosition:
        """
        单轴相对运动

        Args:
            joint: 关节编号 (1-6)
            delta: 相对运动量 (度)
            speed: 临时速度
        """
        self.request_position()
        joints = list(self._position.joints.as_tuple())
        joints[joint - 1] += delta
        return self.move_joints(*joints, speed=speed)

    # ─── 笛卡尔空间运动 ──────────────────────────────────────

    def move_cartesian(
        self,
        x: float, y: float, z: float,
        rx: float = 0, ry: float = 0, rz: float = 0,
        j7: float = 0, j8: float = 0, j9: float = 0,
        speed: Optional[float] = None,
    ) -> RobotPosition:
        """
        笛卡尔空间点到点运动 (MJ 指令，Teensy 做 IK)

        Args:
            x, y, z: 目标位置 (mm)
            rx, ry, rz: 目标姿态 (度)
        """
        params = self._build_speed_params(speed)
        wc = self._calc_wrist_config()

        cmd = (
            f"MJ"
            f"X{x}Y{y}Z{z}Rz{rz}Ry{ry}Rx{rx}"
            f"J7{j7}J8{j8}J9{j9}"
            f"{params}W{wc}Lm{self._loop_mode}\n"
        )

        response = self._send_main(cmd)
        self._handle_response(response)
        return self._position

    def move_linear(
        self,
        x: float, y: float, z: float,
        rx: float = 0, ry: float = 0, rz: float = 0,
        j7: float = 0, j8: float = 0, j9: float = 0,
        speed: Optional[float] = None,
        rounding: float = 0,
        disable_wrist: bool = False,
    ) -> RobotPosition:
        """
        笛卡尔空间直线运动 (ML 指令)

        Args:
            x, y, z: 目标位置 (mm)
            rx, ry, rz: 目标姿态 (度)
            rounding: 拐角圆滑半径
            disable_wrist: 是否禁用腕部旋转
        """
        params = self._build_speed_params(speed, is_linear=True)
        wc = self._calc_wrist_config()
        dis_wrist = "1" if disable_wrist else "0"

        cmd = (
            f"ML"
            f"X{x}Y{y}Z{z}Rz{rz}Ry{ry}Rx{rx}"
            f"J7{j7}J8{j8}J9{j9}"
            f"{params}Rnd{rounding}W{wc}Lm{self._loop_mode}Q{dis_wrist}\n"
        )

        response = self._send_main(cmd)
        self._handle_response(response)
        return self._position

    # ─── 舵机/夹爪控制 ───────────────────────────────────────

    def set_servo(self, servo_num: int, position: int):
        """
        控制舵机 (通过 Nano 扩展板)

        Args:
            servo_num: 舵机编号 (0-7)
            position: 舵机角度 (0-180)
        """
        if not 0 <= servo_num <= 7:
            raise ValueError(f"Servo number must be 0-7, got {servo_num}")
        if not 0 <= position <= 180:
            raise ValueError(f"Servo position must be 0-180, got {position}")

        cmd = f"SV{servo_num}P{position}\n"
        self._send_nano(cmd)

    def gripper_open(self, servo: int = 0, position: int = 90):
        """张开夹爪"""
        self.set_servo(servo, position)
        logger.info(f"Gripper opened (servo {servo} → {position})")

    def gripper_close(self, servo: int = 0, position: int = 30):
        """闭合夹爪"""
        self.set_servo(servo, position)
        logger.info(f"Gripper closed (servo {servo} → {position})")

    # ─── 数字 IO ────────────────────────────────────────────

    def set_output(self, pin: int, state: bool):
        """
        设置数字输出 (通过 Nano)

        Args:
            pin: 引脚编号 (8-13)
            state: True=HIGH, False=LOW
        """
        cmd = f"{'ON' if state else 'OF'}X{pin}\n"
        self._send_nano(cmd)

    def read_input(self, pin: int) -> bool:
        """
        读取数字输入 (通过 Nano)

        Args:
            pin: 引脚编号 (2-7)

        Returns:
            True=HIGH, False=LOW
        """
        cmd = f"JFX{pin}T\n"
        response = self._send_nano(cmd)
        return response.strip() == "T"

    # ─── 系统指令 ────────────────────────────────────────────

    def test_connection(self) -> bool:
        """测试连接 (发 TM 指令)"""
        try:
            response = self._send_main("TM hello\n", parse=False)
            return "hello" in response
        except Exception:
            return False

    def enable_freedrive(self):
        """使能拖动示教模式 (TL)"""
        self._send_main("TL\n", parse=False)
        logger.info("Freedrive enabled")

    def servo_on(self):
        """使能伺服 (SE)"""
        self._send_main("SE\n", parse=False)
        logger.info("Servos enabled")

    def servo_off(self):
        """释放伺服 (RE)"""
        self._send_main("RE\n", parse=False)
        logger.info("Servos released")

    # ─── 标定 ────────────────────────────────────────────────

    def calibrate_joint(self, joint: int, cal_offsets: dict = None):
        """
        校准单个关节 (LL 指令)
        注意: 需要先把机械臂移到限位开关位置

        Args:
            joint: 关节编号 (1-9)
            cal_offsets: 校准偏移量字典，默认全零
        """
        if cal_offsets is None:
            cal_offsets = {k: 0 for k in range(1, 10)}

        flags = ["0"] * 9
        flags[joint - 1] = "1"
        flag_str = "".join(f"{chr(65+i)}{flags[i]}" for i in range(9))
        offset_str = "".join(
            f"{chr(74+i)}{cal_offsets.get(i+1, 0)}" for i in range(9)
        )

        cmd = f"LL{flag_str}{offset_str}\n"
        response = self._send_main(cmd)
        self._handle_response(response)
        logger.info(f"Joint {joint} calibrated")

    # ─── 私有方法 ────────────────────────────────────────────

    def _send_main(self, cmd: str, parse: bool = True) -> str:
        """发送指令到主控 (Teensy)"""
        if not self._ser or not self._ser.is_open:
            raise PickerError("Main serial not connected")

        logger.debug(f"TX → {cmd.strip()}")
        self._ser.write(cmd.encode())
        self._ser.flushInput()
        time.sleep(0.1)
        response = self._ser.readline().decode("utf-8", errors="ignore").strip()
        logger.debug(f"RX ← {response}")
        return response

    def _send_nano(self, cmd: str) -> str:
        """发送指令到 Nano 扩展板"""
        target = self._ser_nano if self._ser_nano else self._ser
        if not target or not target.is_open:
            raise PickerError("Nano serial not connected")

        logger.debug(f"TX(nano) → {cmd.strip()}")
        target.write(cmd.encode())
        time.sleep(0.1)
        response = target.readline().decode("utf-8", errors="ignore").strip()
        logger.debug(f"RX(nano) ← {response}")
        return response

    def _build_speed_params(
        self, speed: Optional[float] = None, is_linear: bool = False
    ) -> str:
        """构建速度参数字符串"""
        p = self._motion_params
        spd = speed if speed is not None else p.speed

        if p.speed_type == "seconds":
            prefix = "Ss"
        elif p.speed_type == "mm_per_sec" and is_linear:
            prefix = "Sm"
        else:
            prefix = "Sp"

        return f"{prefix}{spd}Ac{p.accel}Dc{p.decel}Rm{p.ramp}"

    def _calc_wrist_config(self, j5: Optional[float] = None) -> str:
        """计算腕部配置标志"""
        if j5 is None:
            j5 = self._position.joints.j5
        return "F" if j5 > 0 else "N"

    def _check_joint_limits(self, j1, j2, j3, j4, j5, j6):
        """检查关节限位"""
        joints = [j1, j2, j3, j4, j5, j6]
        for i, val in enumerate(joints, 1):
            lo, hi = self.JOINT_LIMITS[i]
            if val < lo or val > hi:
                raise PickerAxisLimitError(
                    {i: True, **{j: False for j in range(1, 7) if j != i}}
                )

    def _parse_position(self, response: str):
        """解析位置响应"""
        try:
            pos = self._position

            idx_a = response.index('A')
            idx_b = response.index('B')
            idx_c = response.index('C')
            idx_d = response.index('D')
            idx_e = response.index('E')
            idx_f = response.index('F')
            idx_g = response.index('G')
            idx_h = response.index('H')
            idx_i = response.index('I')
            idx_j = response.index('J')
            idx_k = response.index('K')
            idx_l = response.index('L')
            idx_m = response.index('M')
            idx_n = response.index('N')
            idx_o = response.index('O')
            idx_p = response.index('P')
            idx_q = response.index('Q')
            idx_r = response.index('R')

            pos.joints.j1 = float(response[idx_a+1:idx_b])
            pos.joints.j2 = float(response[idx_b+1:idx_c])
            pos.joints.j3 = float(response[idx_c+1:idx_d])
            pos.joints.j4 = float(response[idx_d+1:idx_e])
            pos.joints.j5 = float(response[idx_e+1:idx_f])
            pos.joints.j6 = float(response[idx_f+1:idx_g])

            pos.pose.x = float(response[idx_g+1:idx_h])
            pos.pose.y = float(response[idx_h+1:idx_i])
            pos.pose.z = float(response[idx_i+1:idx_j])
            pos.pose.rz = float(response[idx_j+1:idx_k])
            pos.pose.ry = float(response[idx_k+1:idx_l])
            pos.pose.rx = float(response[idx_l+1:idx_m])

            pos.speed_violation = response[idx_m+1:idx_n]
            pos.debug = response[idx_n+1:idx_o]
            pos.flag = response[idx_o+1:idx_p]
            pos.j7 = float(response[idx_p+1:idx_q])
            pos.j8 = float(response[idx_q+1:idx_r])
            pos.j9 = float(response[idx_r+1:])

        except (ValueError, IndexError) as e:
            logger.warning(f"Failed to parse position: {response} ({e})")

    def _handle_response(self, response: str):
        """处理响应，区分成功/错误"""
        if not response:
            raise PickerError("No response from Picker")

        if response.startswith('E'):
            self._handle_error(response)
        else:
            self._parse_position(response)

    def _handle_error(self, response: str):
        """处理错误响应"""
        if len(response) > 1 and response[1] == 'L':
            # 轴限位错误: EL000000000
            limits = {}
            for i in range(1, 10):
                idx = i + 1
                if idx < len(response):
                    limits[i] = response[idx] == '1'
            raise PickerAxisLimitError(limits)
        else:
            raise PickerError(f"Robot error: {response}")

    # ─── 上下文管理器 ────────────────────────────────────────

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    def __repr__(self):
        status = "connected" if self.is_connected else "disconnected"
        return f"PickerDriver({self.main_port}, {status})"
