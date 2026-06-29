#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import threading 
import os
import rospy
from std_msgs.msg import String


class VoiceTaskInputNode:
    """
    语音任务输入节点。

    功能：
    1. 从 launch 参数 audio_path 读取音频文件并转写；
    2. 或者运行时订阅 /voice_audio_path，收到音频路径后转写；
    3. 将识别到的任务文本发布到 /voice_task_text；
    4. 如果识别结果为空，可在终端手动输入任务文本作为兜底。
    """

    def __init__(self):
        rospy.init_node("voice_task_input_node", anonymous=False)

        self.audio_path = rospy.get_param("~audio_path", "")
        self.audio_path_topic = rospy.get_param("~audio_path_topic", "/voice_audio_path")
        self.task_text_topic = rospy.get_param("~task_text_topic", "/voice_task_text")

        self.model_name = rospy.get_param("~model_name", "base")
        self.language = rospy.get_param("~language", "zh")
        self.keep_alive = rospy.get_param("~keep_alive", True)

        self.terminal_fallback = rospy.get_param("~terminal_fallback", True)
        self.terminal_prompt = rospy.get_param("~terminal_prompt", "语音识别为空，请手动输入任务：")

        rospy.loginfo("[VoiceTaskInput] loading Whisper model: %s" % self.model_name)

        try:
            import whisper
            self.model = whisper.load_model(self.model_name)
        except Exception as e:
            rospy.logerr("[VoiceTaskInput] failed to load Whisper: %s" % str(e))
            rospy.logerr("[VoiceTaskInput] install with: pip3 install --user openai-whisper")
            raise

        # 繁体转简体。没有 OpenCC 时不影响主流程。
        self.converter = None
        try:
            from opencc import OpenCC
            self.converter = OpenCC("t2s")
            rospy.loginfo("[VoiceTaskInput] OpenCC enabled: traditional -> simplified")
        except Exception:
            rospy.logwarn("[VoiceTaskInput] OpenCC not found, simplified conversion disabled")
            rospy.logwarn("[VoiceTaskInput] install with: pip3 install --user opencc-python-reimplemented")

        # latch=True：后启动的智能分拣节点也能拿到最近一次任务文本
        self.pub = rospy.Publisher(
            self.task_text_topic,
            String,
            queue_size=1,
            latch=True
        )

        self.path_sub = rospy.Subscriber(
            self.audio_path_topic,
            String,
            self.audio_path_callback,
            queue_size=1
        )

        rospy.loginfo("[VoiceTaskInput] publish task text to: %s" % self.task_text_topic)
        rospy.loginfo("[VoiceTaskInput] listen audio path from: %s" % self.audio_path_topic)

    def simplify_chinese(self, text):
        text = (text or "").strip()
        if self.converter is not None:
            try:
                text = self.converter.convert(text)
            except Exception as e:
                rospy.logwarn("[VoiceTaskInput] OpenCC conversion failed: %s" % str(e))
        return text

    def transcribe_audio(self, audio_path):
        if not audio_path:
            rospy.logwarn("[VoiceTaskInput] empty audio path")
            return ""

        audio_path = os.path.expanduser(audio_path)

        if not os.path.exists(audio_path):
            rospy.logerr("[VoiceTaskInput] audio file not found: %s" % audio_path)
            return ""

        rospy.loginfo("[VoiceTaskInput] transcribing audio: %s" % audio_path)

        try:
            result = self.model.transcribe(
                audio_path,
                language=self.language,
                task="transcribe",
                fp16=False,
                initial_prompt="以下是混合普通话和英文语音，请输出简体中文和英文。"
            )

            text = result.get("text", "").strip()
            text = self.simplify_chinese(text)

            rospy.loginfo("[VoiceTaskInput] recognized text: %s" % text)
            return text

        except Exception as e:
            rospy.logerr("[VoiceTaskInput] transcription failed: %s" % str(e))
            return ""

    def get_terminal_task_text(self):
        if not self.terminal_fallback:
            return ""

        try:
            text = input(self.terminal_prompt).strip()
            text = self.simplify_chinese(text)
            return text
        except Exception as e:
            rospy.logwarn("[VoiceTaskInput] terminal input failed: %s" % str(e))
            return ""

    def publish_task_text(self, text):
        text = (text or "").strip()
        if not text:
            rospy.logwarn("[VoiceTaskInput] empty task text, not publishing")
            return False

        self.pub.publish(String(data=text))
        rospy.set_param("/latest_voice_task_text", text)

        rospy.loginfo("[VoiceTaskInput] published task text: %s" % text)
        return True

    def process_audio_path(self, audio_path):
        text = self.transcribe_audio(audio_path)

        if not text:
            rospy.logwarn("[VoiceTaskInput] ASR result is empty")
            text = self.get_terminal_task_text()

        return self.publish_task_text(text)

    def audio_path_callback(self, msg):
        rospy.loginfo("[VoiceTaskInput] received audio path topic: %s" % msg.data)
        self.process_audio_path(msg.data)

    def run(self):
        # 情况1: launch 里直接给了 audio_path，先处理它
        if self.audio_path:
            self.process_audio_path(self.audio_path)
            if not self.keep_alive:
                rospy.sleep(1.0)
                return

        # 情况2: 进入"按住 Enter 录音"模式（默认开启）
        hold_mode = rospy.get_param("~hold_to_record", True)
        if hold_mode:
            try:
                self.run_hold_record_mode()
                return  # 按住模式内部自己会 rospy.spin()
            except Exception as e:
                rospy.logwarn("[VoiceTaskInput] 按住录音模式失败: %s，回退到 topic 监听" % str(e))

        # 情况3: 回退到原来的 topic 监听模式
        if self.audio_path == "":
            rospy.loginfo("[VoiceTaskInput] no audio_path param provided")
            rospy.loginfo("[VoiceTaskInput] send audio path by:")
            rospy.loginfo('rostopic pub /voice_audio_path std_msgs/String "data: \'/home/robotics/task.wav\'"')

        if self.keep_alive:
            rospy.spin()
        else:
            rospy.sleep(1.0)
            
    def run_hold_record_mode(self):
        """
        按住 Enter 键开始录音，松开 Enter 键结束录音并识别。
        支持连续多段录音。
        """
        from pynput import keyboard
        import sounddevice as sd
        import numpy as np
        import wave
        import tempfile

        rospy.loginfo("[VoiceTaskInput] =========================================")
        rospy.loginfo("[VoiceTaskInput] 按住 Enter 键开始录音，松开 Enter 结束")
        rospy.loginfo("[VoiceTaskInput] 请按住 Enter 对着麦克风说出任务指令...")
        rospy.loginfo("[VoiceTaskInput] =========================================")

        frames = []
        recording = False
        stream = None
        sample_rate = 16000
        channels = 1

        def audio_callback(indata, frame_count, time_info, status):
            if recording:
                frames.append(indata.copy())

        def on_press(key):
            nonlocal recording, stream
            if key == keyboard.Key.enter and not recording:
                recording = True
                frames.clear()
                stream = sd.InputStream(
                    samplerate=sample_rate,
                    channels=channels,
                    dtype=np.int16,
                    callback=audio_callback
                )
                stream.start()
                rospy.loginfo("[VoiceTaskInput] 🎤 开始录音...")

        def on_release(key):
            nonlocal recording, stream
            if key == keyboard.Key.enter and recording:
                recording = False
                if stream:
                    stream.stop()
                    stream.close()
                    stream = None

                if len(frames) == 0:
                    return True

                # 合并音频帧
                data = np.concatenate(frames, axis=0)
                duration = len(data) / sample_rate

                if duration < 0.5:
                    rospy.logwarn("[VoiceTaskInput] 录音太短 (%.1fs)，请长按 Enter 多说话" % duration)
                    return True

                # 保存临时 wav
                tmp_path = os.path.join(
                    tempfile.gettempdir(),
                    "hold_record_%d.wav" % rospy.Time.now().to_nsec()
                )
                with wave.open(tmp_path, 'wb') as wf:
                    wf.setnchannels(channels)
                    wf.setsampwidth(2)
                    wf.setframerate(sample_rate)
                    wf.writeframes(data.tobytes())

                rospy.loginfo("[VoiceTaskInput] ✅ 录音结束 (%.1fs)，正在识别..." % duration)

                # 在后台线程做 Whisper 识别，避免阻塞键盘监听
                threading.Thread(target=self.process_audio_path, args=(tmp_path,)).start()

            return True

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        rospy.loginfo("[VoiceTaskInput] 已进入按住录音模式，Ctrl+C 退出节点...")
        rospy.spin()

        listener.stop()


if __name__ == "__main__":
    node = VoiceTaskInputNode()
    node.run()
