# 🤖 Picker V2 Solo Demo 计划

## Picker V2 能力总结

| 项目 | 参数 |
|------|------|
| 自由度 | 6-DOF |
| 主控 | Teensy 4.1 |
| 电机 | ZDT 闭环步进 (16384 CPR 编码器) |
| 末端 | Arduino Nano × 8路舵机 (夹爪/工具) |
| 上位机 | AR4 HMI 5.1 (Python GUI，含运动学/G-Code/Vision) |
| 通信 | Serial 115200 |
| DH参数 | 已知 (d: 139/55/240/55/0/0, a: 0/-90/0/-90/90/-90) |
| 关节范围 | J1 ±163°, J2 -60~90°, J3 -90~85°, J4 ±173°, J5 ±108°, J6 ±180° |

---

## 💡 推荐方案: "LLM 驱动的智能抓取助手"

### 概念
用自然语言控制 Picker：对着麦克风说"帮我把红色方块放到右边盒子里"，LLM 理解指令 → 摄像头识别目标 → 规划轨迹 → Picker 执行。**让机械臂真正"听懂人话"。**

### 为什么有趣
1. **LLM + 机器人** — 当前最热的研究方向，展示性极强
2. **端到端** — 语音 → 理解 → 视觉 → 规划 → 执行，完整链路
3. **Picker 的硬件完全够** — 6DOF + 夹爪 + AR4 已有 Vision 模块
4. **容易出效果** — 抓取几个彩色方块就很有冲击力
5. **可扩展** — 后续加 Dummy 做双臂协作无缝衔接

### 系统架构

```
┌─────────────┐  语音  ┌──────────────┐  文字  ┌─────────────┐
│  麦克风      │ ─────→ │  Whisper     │ ─────→ │  LLM        │
│  (Mac内置)   │        │  语音转文字   │        │  意图解析    │
└─────────────┘        └──────────────┘        │  动作规划    │
                                                └──────┬──────┘
                                                       │ JSON
                                                       ▼
┌─────────────┐  bbox  ┌──────────────┐  串口  ┌──────────────┐
│  USB 摄像头  │ ─────→ │  Python 中控  │ ─────→ │  Picker      │
│  俯视棋盘    │        │  坐标转换     │        │  Teensy 4.1  │
└─────────────┘        │  轨迹生成     │        │  执行抓放     │
                        └──────────────┘        └──────────────┘
```

### 工作流程

```
用户: "把蓝色的积木放到盘子里"
  ↓
Whisper STT → "把蓝色的积木放到盘子里"
  ↓
LLM 解析 → {action: "pick_and_place", object: "blue_block", target: "plate"}
  ↓
摄像头识别 → blue_block @ pixel(320, 240), plate @ pixel(480, 300)
  ↓
像素→世界坐标 → blue_block @ (150mm, 80mm, 0mm), plate @ (200mm, -50mm, 0mm)
  ↓
Picker 执行:
  1. MoveJ → 蓝色积木上方 (安全高度)
  2. MoveL → 下降到抓取高度
  3. 夹爪闭合 (SV0P30)
  4. MoveL → 抬起
  5. MoveJ → 盘子上方
  6. MoveL → 下降到放置高度
  7. 夹爪张开 (SV0P90)
  8. MoveL → 抬起回安全位
  ↓
语音反馈: "好的，蓝色积木已经放到盘子里了"
```

### 开发阶段

#### Phase 1: Picker 驱动层 (1 天)
- [ ] Python 串口驱动，封装 AR4 协议
  - 发送关节角: 基于 AR4 HMI 源码逆向协议格式
  - 控制舵机: `SV<n>P<pos>\n` 
  - 读取位置: 解析编码器反馈
- [ ] 实现 `move_j(j1..j6)`, `move_l(x,y,z,rx,ry,rz)`, `gripper(open/close)`
- [ ] 测试基本运动: home → 几个预设点 → 回 home

#### Phase 2: 视觉感知 (1-2 天)
- [ ] USB 摄像头俯拍工作区
- [ ] OpenCV 颜色分割识别彩色积木 (HSV 阈值)
- [ ] 手眼标定: 像素坐标 → 机械臂基坐标系
  - 简单4点标定法即可
- [ ] 物体检测: 返回 `{name, color, position_mm}`

#### Phase 3: LLM 意图解析 (1 天)
- [ ] Whisper 语音识别 (可直接用 Mac 麦克风)
- [ ] LLM Function Calling 解析指令:
  ```python
  tools = [{
      "name": "pick_and_place",
      "params": {"object": "str", "target": "str"}
  }, {
      "name": "point_at", 
      "params": {"object": "str"}
  }, {
      "name": "wave",  # 打招呼
      "params": {}
  }, {
      "name": "sort_by_color",
      "params": {"colors": ["red", "blue", "green"]}
  }]
  ```
- [ ] TTS 语音反馈 (say 命令 / ElevenLabs)

#### Phase 4: 整合演示 (1 天)
- [ ] 端到端流程打通
- [ ] 加入几个有趣的交互:
  - "数一数桌上有几个积木" (视觉计数)
  - "按颜色排列" (批量分拣)
  - "把所有积木摞起来" (堆叠)
  - "跟我打个招呼" (挥手动作)
- [ ] 录演示视频

### 所需额外硬件
- **USB 摄像头** (普通网络摄像头即可) — 俯拍工作区
- **彩色积木/方块** — 3~5 个不同颜色，2-3cm 大小
- **简单场景** — 白色桌面 + 几个小盒子/盘子作为目标区

### 关键代码模块

```
workspace/picker-demo/
├── driver/
│   ├── picker_serial.py     # Picker 串口驱动
│   ├── ar4_protocol.py      # AR4 通信协议解析
│   └── gripper.py           # 舵机夹爪控制
├── vision/
│   ├── camera.py            # 摄像头采集
│   ├── detector.py          # 物体检测 (颜色分割)
│   └── calibration.py       # 手眼标定
├── brain/
│   ├── llm_planner.py       # LLM 意图解析
│   ├── speech.py            # STT + TTS
│   └── task_executor.py     # 任务执行状态机
├── config.py                # 机械臂参数/标定数据
├── main.py                  # 主入口
└── README.md
```

---

## 🎯 备选方案

### 方案 B: "Picker 画家" — 末端夹笔画画
- 末端夹持马克笔，在纸上画简笔画
- 输入图片 → 边缘检测 → 路径规划 → 画出来
- 不需要摄像头，纯运动控制
- 适合展示运动学精度

### 方案 C: "自动分拣线" — 模拟工业场景
- 传送带 (可用倾斜板代替) 送来物体
- Picker 按颜色/大小分拣到不同区域
- 偏工业风，适合技术展示

---

*创建日期: 2026-03-12*
*状态: 规划中 — 仅 Picker V2*
*后续: 拿到 Dummy 后可升级为双臂协作*
