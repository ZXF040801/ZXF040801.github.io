import logging

import numpy as np
import webrtcvad
import soundfile as sf
from datetime import datetime
from collections import deque

# 参数配置
SAMPLE_RATE = 16000  # 采样率 (必须为8000, 16000, 32000, 48000)
FRAME_DURATION = 10  # 每帧持续时间（毫秒）
CHANNELS = 1  # 单声道
VAD_MODE = 2  # VAD检测模式（0-3，3最严格）

# VAD参数
SPEECH_ACTIVATION_THRESHOLD = 20  # 连续检测到语音的帧数阈值
SILENCE_DEACTIVATION_THRESHOLD = 10  # 连续静音帧数阈值（约600ms）


class VoiceActivityDetector:
    def __init__(self):
        self.logger = logging.getLogger("voice_interface_node.VAD")
        self.vad = webrtcvad.Vad(VAD_MODE)
        self.frame_size = int(SAMPLE_RATE * FRAME_DURATION / 1000)
        self.audio_buffer = []
        self.is_recording = False
        self.silence_counter = 0
        self.speech_counter = 0
        self.pre_frames = deque(maxlen=SPEECH_ACTIVATION_THRESHOLD)

    def process_frame(self, frame_bytes):
        """处理音频帧"""
        if self.is_recording:
            if self.vad.is_speech(frame_bytes, SAMPLE_RATE):
                self.silence_counter = 0
                self.audio_buffer.append(frame_bytes)
            else:
                self.silence_counter += 1
                self.audio_buffer.append(frame_bytes)

                if self.silence_counter >= SILENCE_DEACTIVATION_THRESHOLD:
                    self.logger.debug("检测到静音，停止录音...")
                    filename = self._save_recording()
                    self._reset()
                    return filename
        else:
            if self.vad.is_speech(frame_bytes, SAMPLE_RATE):
                self.speech_counter += 1
                self.pre_frames.append(frame_bytes)

                if self.speech_counter >= SPEECH_ACTIVATION_THRESHOLD:
                    self.logger.debug("检测到语音，开始录音...")
                    self.is_recording = True
                    self.audio_buffer.extend(self.pre_frames)
                    self.audio_buffer.append(frame_bytes)
                    self.speech_counter = 0
                    self.pre_frames.clear()
            else:
                self.speech_counter = 0
                self.pre_frames.clear()
        return None

    def _save_recording(self):
        """保存录音文件"""
        if not self.audio_buffer:
            return

        # 合并音频数据
        audio_data = b''.join(self.audio_buffer)
        np_audio = np.frombuffer(audio_data, dtype=np.int16)

        # 生成文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"recording_{timestamp}.wav"

        # 保存为WAV文件
        sf.write(filename, np_audio, SAMPLE_RATE, subtype='PCM_16')
        self.logger.debug(f"录音已保存：{filename}")
        return filename

    def _reset(self):
        """重置状态"""
        self.is_recording = False
        self.silence_counter = 0
        self.speech_counter = 0
        self.audio_buffer = []


