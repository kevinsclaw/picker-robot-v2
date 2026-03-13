# 🤖 Picker × Dummy 联动 Demo 计划

## 项目概览

### Picker Robot V2
- **架构**: 6DOF 机械臂，基于 AR4 开源项目
- **主控**: Teensy 4.1 (Arduino)
- **电机**: ZDT 闭环步进电机 (Emm V5.0 / X42 V2)，通过脉冲+方向控制
- **通信**: 串口协议，上位机 Python/GUI (AR4 HMI 5.1)
- **末端**: Arduino Nano 控制的8路舵机扩展板（夹爪/工具）
- **特点**: 工业风，步进电机大扭矩，有完整 URDF 和 DH 参数

### Dummy V2
- **架构**: 6DOF 桌面机械臂，基于稚晖君设计
- **主控**: STM32F4 + FreeRTOS
- **电机**: CtrlStep 闭环步进电机，CAN 总线通信
- **通信**: USB-CDC / UART ASCII 协议，支持 MoveJ / MoveL
- **末端**: CAN 总线驱动的夹爪 (DummyHand)
- **附加**: OLED 显示屏、MPU6050 IMU、RGB 灯效、ESP32 IoT 模块
- **特点**: 桌面级精巧，有正/逆运动学求解器，支持连续轨迹模式

---

## 💡 Demo 方案: "AI 书法家" — 双臂协作写毛笔字

### 概念
Dummy 作为"手"握毛笔在纸上写字，Picker 作为"助手"负责蘸墨、递纸、换笔。两台机械臂通过一个 Python 中控协调，接收用户输入的汉字，自动规划笔画轨迹，协作完成书写。

### 为什么有趣
1. **双臂协作** — 不是各干各的，而是有交接动作（Picker 把毛笔递给 Dummy）
2. **融合两种控制体系** — Picker 的脉冲步进 + Dummy 的 CAN 闭环
3. **视觉反馈** — Dummy 的 OLED 实时显示当前写的字和进度
4. **IoT 联动** — Dummy 的 ESP32 提供 Web 界面，用户手机输入想写的字
5. **IMU 辅助** — Dummy 的 MPU6050 检测桌面是否水平，自动补偿

### 系统架构

```
┌──────────────┐     WiFi/HTTP      ┌──────────────┐
│  手机/浏览器  │ ──────────────────→ │  ESP32 IoT   │
│  输入汉字     │                     │  Web Server  │
└──────────────┘                     └──────┬───────┘
                                           │ UART
                                           ▼
┌──────────────┐     USB-CDC      ┌──────────────────┐
│  Python 中控  │ ───────────────→ │  Dummy (STM32F4) │
│  轨迹规划     │                  │  写字执行         │
│  双臂协调     │                  │  OLED + RGB 反馈  │
└──────┬───────┘                  └──────────────────┘
       │ Serial
       ▼
┌──────────────────┐
│  Picker (Teensy)  │
│  蘸墨 / 递笔      │
│  换纸 / 辅助      │
└──────────────────┘
```

### 开发阶段

#### Phase 1: 通信打通 (1-2 天)
- [ ] 用 Python 分别连接 Picker (Serial) 和 Dummy (USB-CDC)
- [ ] 封装统一的 `RobotController` 类，屏蔽底层协议差异
- [ ] 验证 Dummy 的 MoveJ / MoveL 指令
- [ ] 验证 Picker 的舵机控制指令 (`SV<n>P<pos>`)
- [ ] 测试 Dummy OLED 显示自定义内容

#### Phase 2: 单臂轨迹 (2-3 天)
- [ ] 实现汉字笔画 SVG → 3D 轨迹点转换
- [ ] 用 Dummy 的 MoveL (笛卡尔空间) 实现单笔画描绘
- [ ] 调试连续轨迹模式 (`COMMAND_CONTINUES_TRAJECTORY`)
- [ ] 加入抬笔/落笔动作（Z 轴升降）

#### Phase 3: 双臂协作 (2-3 天)
- [ ] 定义蘸墨工作流: Picker 夹取毛笔 → 蘸墨 → 递给 Dummy
- [ ] 实现安全的交接握手协议（确认到位再松手）
- [ ] Picker 换纸流程: 推走写好的纸，铺上新纸
- [ ] 双臂避碰: 简单的时序互锁

#### Phase 4: 整合上线 (1-2 天)
- [ ] ESP32 Web 界面: 输入汉字 + 选字体风格
- [ ] OLED 实时状态显示
- [ ] RGB 灯效: 写字时呼吸灯，完成时彩虹
- [ ] 录一个完整演示视频

### 技术要点

| 模块 | 技术 | 备注 |
|------|------|------|
| 笔画解析 | Python + SVG 路径 | 开源汉字笔顺数据 |
| 轨迹规划 | Dummy MoveL 连续模式 | 200Hz 控制循环 |
| 协作同步 | Python 状态机 | 事件驱动 |
| 末端工具 | Picker Nano 舵机 | 夹爪开合 |
| 用户界面 | ESP32 Mongoose HTTP | 手机访问 |

### 所需硬件补充
- 毛笔 (或马克笔，更容易控制)
- 墨水/墨盒
- 固定夹具 (3D 打印，让 Picker 能稳定夹持笔)
- 纸张固定台

---

## 🎯 备选 Demo (如果书法方案太复杂)

### 方案 B: "叠叠乐" — 双臂积木堆叠
- Picker 从料仓取积木放到中间区域
- Dummy 精确堆叠成指定形状
- 手机选择目标图案，ESP32 传输
- 简单但视觉效果好

### 方案 C: "调酒师" — 自动调饮料
- Picker 拿杯子、递杯子
- Dummy 操作量杯倒液体
- 手机点单，OLED 显示配方进度
- 观赏性极强，适合展示

---

## 📁 文件结构规划

```
workspace/
├── picker-dummy-demo/
│   ├── controller/
│   │   ├── picker_driver.py      # Picker 串口驱动
│   │   ├── dummy_driver.py       # Dummy USB-CDC 驱动
│   │   ├── coordinator.py        # 双臂协调器
│   │   └── trajectory.py         # 轨迹规划
│   ├── calligraphy/
│   │   ├── stroke_parser.py      # 笔画解析
│   │   └── fonts/                # 字体数据
│   ├── web/
│   │   └── index.html            # ESP32 Web 界面
│   └── README.md
```

---

*创建日期: 2026-03-12*
*状态: 规划中*
