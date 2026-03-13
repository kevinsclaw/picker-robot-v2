# Picker V2 Python Demo

基于 AR4 v5.1 协议的 Picker Robot V2 Python 驱动和演示程序。

## 文件结构

```
picker-demo/
├── driver/
│   └── picker_serial.py   # 核心驱动 (AR4 串口协议)
├── main.py                 # 演示脚本 (运动/夹爪/抓取放置)
└── README.md
```

## 快速开始

### 1. 安装依赖

```bash
pip install pyserial
```

### 2. 找到串口

```bash
ls /dev/tty.usb*
```

会看到两个端口:
- Teensy 4.1 主控 → 运动指令
- Arduino Nano → 舵机/IO

### 3. 修改端口

编辑 `main.py` 顶部的 `MAIN_PORT` 和 `NANO_PORT`。

### 4. 运行

```bash
cd picker-demo
python main.py
```

## API 速查

```python
from driver.picker_serial import PickerDriver

with PickerDriver("/dev/tty.usbXXX", "/dev/tty.usbYYY") as picker:
    # 速度设置
    picker.set_speed(25)           # 百分比 0-100

    # 位置查询
    pos = picker.request_position()
    print(pos.joints)              # J[0.00, 0.00, ...]
    print(pos.pose)                # P[200.00, 0.00, ...]

    # 关节运动
    picker.move_joints(0, 0, 0, 0, 0, 0)

    # 笛卡尔运动 (点到点)
    picker.move_cartesian(200, 0, 300, 0, 0, 0)

    # 笛卡尔直线运动
    picker.move_linear(200, 100, 300, 0, 0, 0)

    # 夹爪
    picker.gripper_open()
    picker.gripper_close()

    # 舵机
    picker.set_servo(0, 90)

    # IO
    picker.set_output(8, True)
    val = picker.read_input(2)
```

## 串口协议参考

| 指令 | 格式 | 说明 |
|------|------|------|
| RJ | `RJA<j1>B<j2>C<j3>D<j4>E<j5>F<j6>J7<>J8<>J9<>Sp<spd>Ac<>Dc<>Rm<>W<>Lm<>` | 关节运动 |
| MJ | `MJX<x>Y<y>Z<z>Rz<>Ry<>Rx<>...` | 笛卡尔点到点 |
| ML | `MLX<x>Y<y>Z<z>Rz<>Ry<>Rx<>...Rnd<>...Q<>` | 笛卡尔直线 |
| RP | `RP` | 请求位置 |
| SV | `SV<n>P<pos>` | 舵机控制 (Nano) |
| ON/OF | `ONX<pin>` / `OFX<pin>` | 数字输出 (Nano) |
| TL | `TL` | 拖动示教 |
| SE | `SE` | 使能伺服 |
| RE | `RE` | 释放伺服 |
| LL | `LLA<>B<>...` | 校准 |
