"""
Picker Demo 完整集成 — LLM 驱动的智能抓取助手

端到端流程: 语音/文字 → LLM 解析 → 视觉识别 → 机械臂执行 → 语音反馈

使用:
    python app.py                    # 文字输入模式
    python app.py --voice            # 语音模式
    python app.py --no-camera        # 无相机模式 (仅预设动作)
    python app.py --config config.json  # 加载配置
"""

import argparse
import json
import logging
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from driver.picker_serial import PickerDriver
from vision.camera import Camera
from vision.detector import ColorDetector
from vision.calibration import HandEyeCalibration
from brain.llm_planner import LLMPlanner
from brain.task_executor import TaskExecutor, TaskConfig
from brain.speech import SpeechEngine

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("app")


# ============= 🔧 配置区 — 现场修改这里 =============

DEFAULT_CONFIG = {
    # 串口
    "main_port": "/dev/tty.usbmodem_CHANGE_ME",    # 🔧 Teensy 串口
    "nano_port": "/dev/tty.usbmodem_CHANGE_ME_2",  # 🔧 Nano 串口

    # 相机
    "camera_device": 0,          # 0=内置, 1=USB, "gemini2"=深度相机
    "camera_width": 640,
    "camera_height": 480,

    # LLM
    "llm_provider": "openai",    # "openai" | "bedrock"
    "llm_model": "gpt-4o-mini",

    # 语音
    "stt_engine": "none",        # "whisper_local" | "whisper_api" | "none"
    "tts_engine": "macos",       # "macos" | "edge" | "none"

    # 标定文件
    "calibration_file": "calibration.json",

    # 任务参数 (🔧 全部需要现场调)
    "task": {
        "safe_z": 300,
        "pick_z": 30,
        "place_z": 30,
        "approach_z": 80,
        "move_speed": 25,
        "approach_speed": 15,
        "gripper_open_pos": 90,
        "gripper_close_pos": 30,
        "gripper_servo": 0,
    }
}

# ====================================================


def load_config(config_path: str = None) -> dict:
    """加载配置"""
    config = DEFAULT_CONFIG.copy()
    if config_path and os.path.exists(config_path):
        with open(config_path) as f:
            user_config = json.load(f)
        config.update(user_config)
        logger.info(f"Config loaded from {config_path}")
    return config


def run_app(config: dict, use_voice: bool = False, use_camera: bool = True):
    """运行主应用"""

    print()
    print("╔══════════════════════════════════════════╗")
    print("║   🤖 Picker 智能抓取助手                ║")
    print("║   说句话，让机械臂帮你干活               ║")
    print("╚══════════════════════════════════════════╝")
    print()

    # ─── 初始化组件 ──────────────────────────────────────

    # 1. 机械臂
    print("🔌 连接 Picker...")
    picker = PickerDriver(config["main_port"], config.get("nano_port"))
    picker.connect()
    picker.set_speed(config["task"]["move_speed"])
    pos = picker.request_position()
    print(f"   当前位置: {pos.joints}")

    # 2. 相机 (可选)
    camera = None
    detector = None
    calibration = None

    if use_camera:
        print("📷 启动相机...")
        try:
            camera = Camera(
                device=config["camera_device"],
                width=config["camera_width"],
                height=config["camera_height"],
            )
            camera.start()
            print("   相机就绪")

            # 检测器
            detector = ColorDetector(min_area=500, use_presets=True)
            print(f"   颜色检测: {[c.name for c in detector._colors]}")

            # 标定
            calib_file = config["calibration_file"]
            if os.path.exists(calib_file):
                calibration = HandEyeCalibration()
                calibration.load(calib_file)
                print(f"   标定已加载: {calib_file} (误差 {calibration.reprojection_error:.2f}mm)")
            else:
                print(f"   ⚠️ 标定文件不存在: {calib_file}")
                print(f"   运行 test_vision.py 进行标定")

        except Exception as e:
            print(f"   ⚠️ 相机初始化失败: {e}")
            camera = None

    # 3. LLM
    print("🧠 初始化 LLM...")
    planner = LLMPlanner(
        provider=config["llm_provider"],
        model=config.get("llm_model"),
    )
    print(f"   模型: {planner.model}")

    # 4. 语音
    speech = None
    if use_voice or config["tts_engine"] != "none":
        print("🎤 初始化语音...")
        speech = SpeechEngine(
            stt_engine=config["stt_engine"] if use_voice else "none",
            tts_engine=config["tts_engine"],
        )

    # 5. 任务执行器
    task_config = TaskConfig(**config.get("task", {}))
    executor = TaskExecutor(
        picker=picker,
        camera=camera,
        detector=detector,
        calibration=calibration,
        config=task_config,
        speech=speech,
    )

    # ─── 主循环 ──────────────────────────────────────────

    print()
    print("=" * 50)
    print("就绪! 输入指令 (输入 'q' 退出, 'h' 帮助)")
    print("=" * 50)
    print()

    while True:
        try:
            # 获取指令
            if use_voice and speech:
                text = speech.listen(duration=5, prompt="🎤 请说...")
                if text:
                    print(f"   你说: {text}")
            else:
                text = input("你> ").strip()

            if not text:
                continue
            if text.lower() == 'q':
                break
            if text.lower() == 'h':
                _print_help()
                continue
            if text.lower() == 'pos':
                pos = picker.request_position()
                print(f"   关节: {pos.joints}")
                print(f"   位姿: {pos.pose}")
                continue
            if text.lower() == 'scan':
                if camera and detector:
                    frame, depth = camera.read()
                    objects = detector.detect(frame, depth)
                    print(f"   检测到 {len(objects)} 个物体:")
                    for obj in objects:
                        print(f"     {obj}")
                else:
                    print("   相机未连接")
                continue

            # 获取场景信息 (如果有相机)
            scene_objects = None
            if camera and detector and calibration:
                frame, _ = camera.read()
                if frame is not None:
                    detected = detector.detect(frame)
                    scene_objects = [
                        {
                            "name": f"{obj.color}_block",
                            "color": obj.color,
                            "x": calibration.pixel_to_world(obj.cx, obj.cy)[0],
                            "y": calibration.pixel_to_world(obj.cx, obj.cy)[1],
                        }
                        for obj in detected
                    ]

            # LLM 解析
            action = planner.parse(text, scene_objects)
            print(f"   解析: {action}")

            # 执行
            success = executor.execute(action)
            if success:
                print("   ✅ 完成")
            else:
                print("   ❌ 失败")

        except KeyboardInterrupt:
            print("\n中断")
            break
        except Exception as e:
            logger.error(f"Error: {e}")
            print(f"   ❌ 错误: {e}")

    # ─── 清理 ────────────────────────────────────────────

    print("\n🔌 断开连接...")
    if camera:
        camera.stop()
    picker.disconnect()
    print("👋 再见!")


def _print_help():
    print("""
指令示例:
  "把红色积木放到盘子里"    → 抓取放置
  "把蓝色的拿起来"         → 抓取
  "放下"                   → 放置
  "按颜色分拣"             → 自动分拣
  "摞起来"                 → 堆叠
  "数一数有几个"           → 计数
  "打个招呼"               → 挥手
  "回去"                   → 回零位
  "停"                     → 急停

内置命令:
  pos   — 查看当前位置
  scan  — 扫描检测物体
  h     — 帮助
  q     — 退出
""")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Picker 智能抓取助手")
    parser.add_argument("--voice", action="store_true", help="启用语音输入")
    parser.add_argument("--no-camera", action="store_true", help="禁用相机")
    parser.add_argument("--config", type=str, default=None, help="配置文件路径")
    args = parser.parse_args()

    config = load_config(args.config)

    if args.voice:
        config["stt_engine"] = "whisper_local"

    run_app(config, use_voice=args.voice, use_camera=not args.no_camera)
