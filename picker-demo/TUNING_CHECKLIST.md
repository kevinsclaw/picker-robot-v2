# 🔧 现场调试清单

硬件到手后，按这个清单逐步调试。每一步确认 OK 后打勾。

---

## Phase 0: 环境准备

- [ ] `pip install pyserial opencv-python numpy`
- [ ] (语音) `pip install openai-whisper pyaudio`
- [ ] (LLM) `pip install openai` 或 `pip install boto3`
- [ ] (Gemini 2) `pip install pyorbbecsdk`
- [ ] (TTS) `pip install edge-tts`（可选，macOS 自带 say 够用）

---

## Phase 1: Picker 串口连接

### 1.1 找串口
```bash
ls /dev/tty.usb*
```
会看到两个:
- Teensy 4.1 (主控) → 记为 MAIN_PORT
- Arduino Nano (舵机) → 记为 NANO_PORT

🔧 **写入 `app.py` 的 DEFAULT_CONFIG 或创建 config.json**

### 1.2 基础通信测试
```bash
cd ~/.openclaw/workspace/picker-demo
python -c "
from driver.picker_serial import PickerDriver
p = PickerDriver('串口名')
p.connect()
print(p.test_connection())
pos = p.request_position()
print(f'关节: {pos.joints}')
print(f'位姿: {pos.pose}')
p.disconnect()
"
```
- [ ] 连接成功
- [ ] 位置读取正常

### 1.3 运动测试
```bash
python main.py
# 选 1 → 基础关节运动
```
- [ ] J1 转 30° 再回零 — 方向正确
- [ ] 运动平稳，无异响
- [ ] 响应格式解析正确 (打印的关节值合理)

🔧 **如果方向反了**: 检查 `JOINT_LIMITS` 是否需要调整

### 1.4 夹爪测试
```bash
python main.py
# 选 3 → 夹爪测试
```
- [ ] 张开闭合正常

🔧 **需要调的参数** (`task_executor.py → TaskConfig`):
- `gripper_servo`: 夹爪接在 Nano 的哪个舵机口 (0-7)
- `gripper_open_pos`: 完全张开时的舵机角度 (0-180)
- `gripper_close_pos`: 夹紧时的舵机角度 (0-180)
- 抓取力度不够 → 减小 close_pos
- 抓不住 → 检查夹爪机械结构

### 1.5 笛卡尔运动测试
```bash
python main.py
# 选 2 → 笛卡尔运动
```
- [ ] MJ (点到点) 正常
- [ ] ML (直线运动) 正常

🔧 **如果 IK 报错 (超限)**:
- 目标点可能超出工作空间
- 调整测试坐标到合理范围内

---

## Phase 2: 相机视觉

### 2.1 相机连接
```bash
python test_vision.py
# 选 1 → 相机预览
```
- [ ] USB 摄像头 / Gemini 2 画面正常
- [ ] 分辨率正确

### 2.2 颜色调参
```bash
python test_vision.py
# 选 3 → HSV 调参
```
在实际光照条件下，逐个调整每种积木的颜色:

🔧 **需要调的参数** (`detector.py → PRESET_COLORS`):
- [ ] 红色: lower=(...), upper=(...)
- [ ] 蓝色: lower=(...), upper=(...)
- [ ] 绿色: lower=(...), upper=(...)
- [ ] 黄色: lower=(...), upper=(...)

**调参技巧**:
- 白色桌垫效果最好
- 避免阳光直射（色温变化大）
- 哑光积木比亮面好检测
- 调参工具按 's' 打印当前值

🔧 **面积阈值** (`ColorDetector.__init__`):
- `min_area`: 太小检到噪点，太大漏检。建议 300-1000
- `max_area`: 防误检桌面。建议 30000-80000

### 2.3 颜色检测验证
```bash
python test_vision.py
# 选 2 → 颜色检测 (实时)
```
- [ ] 每种颜色积木都能稳定检测
- [ ] 质心位置准确 (标注在物体中心)
- [ ] 没有误检

### 2.4 手眼标定
```bash
python test_vision.py
# 选 5 → 交互式标定 (推荐)
# 或 选 4 → 手动输入标定
```

标定方法:
1. 在工作台 4 个角放标记物 (积木即可)
2. 在画面上点击标记物中心
3. 用 Picker 移到标记物正上方，记录末端坐标
4. 重复 4 次

🔧 **标定精度目标**: 误差 < 5mm (够用了)

- [ ] 4 点标定完成
- [ ] 重投影误差 < 5mm
- [ ] `calibration.json` 已保存
- [ ] 验证: 随机指物体，转换后坐标合理

---

## Phase 3: 运动参数调优

这些参数需要在实际抓取中反复微调:

### 3.1 高度参数 (`TaskConfig`)
```python
safe_z = 300      # 🔧 移动时的安全高度 (不碰任何东西)
approach_z = 80   # 🔧 接近物体时的过渡高度
pick_z = 30       # 🔧 实际抓取高度 (夹爪刚好夹住)
place_z = 30      # 🔧 放置高度
stack_z_step = 25 # 🔧 堆叠每层高度 (=积木高度)
```

**调法**:
1. 手动用 MoveL 慢慢下降到积木高度，记下 Z 值 → `pick_z`
2. `safe_z` 确保不碰工作台上任何东西
3. `approach_z` 是从快速移动切换到慢速的过渡点

### 3.2 速度参数
```python
move_speed = 25     # 🔧 一般移动 (先慢，确认没问题再加)
approach_speed = 15 # 🔧 接近物体时 (要稳)
fast_speed = 40     # 🔧 空载快速移动
```

### 3.3 姿态参数
当前 MoveCartesian 默认姿态是 `(rx=0, ry=0, rz=0)`。
如果夹爪需要竖直朝下抓取，可能需要调整姿态角。

🔧 **如果夹爪朝向不对**: 在 `task_executor.py` 的 `_pick_at` 和 `_place_at` 中修改 rx/ry/rz

### 3.4 预设位置
```python
home_joints = (0, 0, 0, 0, 0, 0)  # 🔧 零位关节角
wave_joints = [...]                 # 🔧 挥手动作序列
nod_joints = [...]                  # 🔧 点头动作序列
sort_zones = {                      # 🔧 分拣目标区域坐标
    "red": (250, -150, 30),
    "blue": (250, -100, 30),
    ...
}
```

---

## Phase 4: LLM + 语音

### 4.1 LLM 连接测试
```python
from brain.llm_planner import LLMPlanner
p = LLMPlanner(provider="openai")  # 或 "bedrock"
action = p.parse("把红色积木放到盘子里")
print(action)
```
- [ ] API 调用正常
- [ ] 解析结果合理

### 4.2 语音测试 (可选)
```python
from brain.speech import SpeechEngine
s = SpeechEngine(stt_engine="whisper_local", tts_engine="macos")
s.speak("你好，我是 Picker")
text = s.listen()
print(f"识别: {text}")
```
- [ ] TTS 播报正常
- [ ] STT 识别中文准确

🔧 **语音识别噪音**: `speech.py` 中 `SILENCE_THRESHOLD = 500`
- 环境噪 → 调大
- 说话声小 → 调小

---

## Phase 5: 端到端集成

```bash
python app.py                # 文字模式
python app.py --voice        # 语音模式
python app.py --no-camera    # 无相机 (仅挥手/点头/回零)
```

### 测试场景
- [ ] "打个招呼" → 挥手
- [ ] "回去" → 回零位
- [ ] "数一数" → 报数
- [ ] "把红色积木放到盘子里" → 完整抓取放置
- [ ] "按颜色分拣" → 批量分拣
- [ ] "摞起来" → 堆叠

---

## 快速参数汇总

创建 `config.json` 保存所有现场参数:

```json
{
    "main_port": "/dev/tty.usbmodemXXX",
    "nano_port": "/dev/tty.usbmodemYYY",
    "camera_device": 1,
    "calibration_file": "calibration.json",
    "task": {
        "safe_z": 300,
        "pick_z": 30,
        "place_z": 30,
        "approach_z": 80,
        "move_speed": 25,
        "approach_speed": 15,
        "gripper_open_pos": 90,
        "gripper_close_pos": 30,
        "gripper_servo": 0
    }
}
```

运行: `python app.py --config config.json`

---

*最后更新: 2026-03-12*
