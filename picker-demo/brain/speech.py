"""
语音模块 — STT (Whisper) + TTS (macOS say / edge-tts)

使用:
    speech = SpeechEngine()

    # 语音识别
    text = speech.listen()          # 阻塞，等用户说完
    print(f"你说: {text}")

    # 语音合成
    speech.speak("好的，已经放好了")

    # 持续监听模式
    for text in speech.listen_continuous():
        print(f"识别: {text}")
        if "停" in text:
            break
"""

import os
import sys
import time
import wave
import logging
import tempfile
import subprocess
import threading
import queue
from typing import Optional, Generator

logger = logging.getLogger(__name__)


class SpeechEngine:
    """
    语音引擎

    STT: OpenAI Whisper (本地 whisper 或 API)
    TTS: macOS say / edge-tts / pyttsx3

    Args:
        stt_engine: "whisper_local" | "whisper_api" | "none"
        tts_engine: "macos" | "edge" | "none"
        whisper_model: Whisper 模型大小 (tiny/base/small/medium/large)
        language: 语音识别语言 ("zh" 中文, "en" 英文)
        tts_voice: TTS 语音名称
    """

    def __init__(
        self,
        stt_engine: str = "whisper_local",
        tts_engine: str = "macos",
        whisper_model: str = "base",
        language: str = "zh",
        tts_voice: Optional[str] = None,
    ):
        self.stt_engine = stt_engine
        self.tts_engine = tts_engine
        self.whisper_model_name = whisper_model
        self.language = language
        self.tts_voice = tts_voice or ("Tingting" if language == "zh" else "Samantha")

        self._whisper_model = None
        self._recorder = None

    # ─── STT 语音识别 ────────────────────────────────────────

    def listen(
        self,
        duration: float = 5.0,
        silence_timeout: float = 2.0,
        prompt: str = "🎤 请说话...",
    ) -> str:
        """
        录音并识别

        Args:
            duration: 最大录音时长 (秒)
            silence_timeout: 静音超时 (秒，检测到静音自动停止)
            prompt: 提示文字

        Returns:
            识别出的文字
        """
        if self.stt_engine == "none":
            return input("(语音禁用) 请输入文字: ").strip()

        print(prompt)

        # 录音
        audio_path = self._record_audio(duration, silence_timeout)
        if audio_path is None:
            return ""

        # 识别
        text = self._transcribe(audio_path)

        # 清理临时文件
        try:
            os.unlink(audio_path)
        except OSError:
            pass

        return text

    def listen_continuous(
        self,
        chunk_duration: float = 5.0,
    ) -> Generator[str, None, None]:
        """
        持续监听模式 (生成器)

        Yields:
            每次识别到的文字
        """
        while True:
            text = self.listen(duration=chunk_duration, prompt="🎤 监听中...")
            if text:
                yield text

    def transcribe_file(self, audio_path: str) -> str:
        """识别音频文件"""
        return self._transcribe(audio_path)

    # ─── TTS 语音合成 ────────────────────────────────────────

    def speak(self, text: str, block: bool = True):
        """
        语音播报

        Args:
            text: 要播报的文字
            block: 是否阻塞等待播报完成
        """
        if self.tts_engine == "none":
            print(f"🔊 {text}")
            return

        if block:
            self._speak_sync(text)
        else:
            t = threading.Thread(target=self._speak_sync, args=(text,), daemon=True)
            t.start()

    def speak_async(self, text: str):
        """非阻塞播报"""
        self.speak(text, block=False)

    # ─── 录音实现 ────────────────────────────────────────────

    def _record_audio(
        self, duration: float, silence_timeout: float
    ) -> Optional[str]:
        """
        使用 PyAudio 录音

        如果没有 PyAudio，回退到 macOS rec/sox 命令
        """
        try:
            return self._record_pyaudio(duration, silence_timeout)
        except ImportError:
            return self._record_sox(duration)

    def _record_pyaudio(self, duration: float, silence_timeout: float) -> str:
        """PyAudio 录音 (带静音检测)"""
        import pyaudio
        import numpy as np

        RATE = 16000
        CHANNELS = 1
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        SILENCE_THRESHOLD = 500  # 🔧 现场调: 环境噪音阈值
        SILENCE_CHUNKS = int(silence_timeout * RATE / CHUNK)

        pa = pyaudio.PyAudio()
        stream = pa.open(
            format=FORMAT, channels=CHANNELS, rate=RATE,
            input=True, frames_per_buffer=CHUNK
        )

        frames = []
        silent_count = 0
        max_chunks = int(duration * RATE / CHUNK)

        for _ in range(max_chunks):
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)

            # 计算音量
            audio_data = np.frombuffer(data, dtype=np.int16)
            volume = np.abs(audio_data).mean()

            if volume < SILENCE_THRESHOLD:
                silent_count += 1
                if silent_count >= SILENCE_CHUNKS and len(frames) > SILENCE_CHUNKS:
                    break
            else:
                silent_count = 0

        stream.stop_stream()
        stream.close()
        pa.terminate()

        # 保存 WAV
        tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        with wave.open(tmp.name, 'wb') as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(2)
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))

        return tmp.name

    def _record_sox(self, duration: float) -> str:
        """macOS sox/rec 命令录音 (备用)"""
        tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        tmp.close()

        try:
            # macOS: 使用 rec (sox) 或 ffmpeg
            subprocess.run(
                ["rec", "-r", "16000", "-c", "1", "-b", "16",
                 tmp.name, "trim", "0", str(duration)],
                capture_output=True, timeout=duration + 2
            )
        except FileNotFoundError:
            # 没有 sox，试 ffmpeg
            try:
                subprocess.run(
                    ["ffmpeg", "-y", "-f", "avfoundation", "-i", ":0",
                     "-t", str(duration), "-ar", "16000", "-ac", "1",
                     tmp.name],
                    capture_output=True, timeout=duration + 2
                )
            except FileNotFoundError:
                logger.error("No recording tool found. Install: brew install sox")
                return None

        return tmp.name

    # ─── Whisper 识别 ────────────────────────────────────────

    def _transcribe(self, audio_path: str) -> str:
        """Whisper 语音识别"""
        if self.stt_engine == "whisper_local":
            return self._transcribe_local(audio_path)
        elif self.stt_engine == "whisper_api":
            return self._transcribe_api(audio_path)
        else:
            return ""

    def _transcribe_local(self, audio_path: str) -> str:
        """本地 Whisper 模型"""
        try:
            import whisper
        except ImportError:
            raise ImportError("pip install openai-whisper")

        if self._whisper_model is None:
            logger.info(f"Loading Whisper model: {self.whisper_model_name}")
            self._whisper_model = whisper.load_model(self.whisper_model_name)

        result = self._whisper_model.transcribe(
            audio_path,
            language=self.language,
            fp16=False,
        )
        text = result["text"].strip()
        logger.info(f"STT: {text}")
        return text

    def _transcribe_api(self, audio_path: str) -> str:
        """OpenAI Whisper API"""
        try:
            from openai import OpenAI
        except ImportError:
            raise ImportError("pip install openai")

        client = OpenAI()
        with open(audio_path, "rb") as f:
            result = client.audio.transcriptions.create(
                model="whisper-1",
                file=f,
                language=self.language,
            )
        text = result.text.strip()
        logger.info(f"STT (API): {text}")
        return text

    # ─── TTS 实现 ────────────────────────────────────────────

    def _speak_sync(self, text: str):
        """同步语音合成"""
        if self.tts_engine == "macos":
            self._speak_macos(text)
        elif self.tts_engine == "edge":
            self._speak_edge(text)

    def _speak_macos(self, text: str):
        """macOS say 命令"""
        try:
            subprocess.run(
                ["say", "-v", self.tts_voice, text],
                capture_output=True, timeout=30
            )
        except Exception as e:
            logger.warning(f"TTS failed: {e}")
            print(f"🔊 {text}")

    def _speak_edge(self, text: str):
        """edge-tts (免费在线 TTS，效果好)"""
        try:
            import edge_tts
            import asyncio

            voice = "zh-CN-XiaoxiaoNeural" if self.language == "zh" else "en-US-AriaNeural"
            tmp = tempfile.NamedTemporaryFile(suffix=".mp3", delete=False)
            tmp.close()

            async def _gen():
                communicate = edge_tts.Communicate(text, voice)
                await communicate.save(tmp.name)

            asyncio.run(_gen())

            # 播放
            subprocess.run(["afplay", tmp.name], capture_output=True, timeout=30)
            os.unlink(tmp.name)

        except ImportError:
            logger.warning("edge-tts not installed, falling back to macOS say")
            self._speak_macos(text)
