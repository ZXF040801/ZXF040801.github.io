#!/home/jetson/miniforge3/envs/py39/bin/python
import time

import numpy as np
import sounddevice as sd
import sys
sys.path.insert(0,'/home/jetson/robot-2/src/log_pkg/scripts')
from log import MyLogger
import os

from log import *
from NLP import nlp, awake

from ASR import ASR2
from VAD import VoiceActivityDetector, CHANNELS, SAMPLE_RATE
from TTS import TTS
from main_pkg.srv import task, taskResponse
from main_pkg.srv import statu, statuResponse
import rospy


is_awake = False
wake_time = time.time()


WAKE_TIMEOUT = 10 # 唤醒超时 10s
INPUT_THRESHOLD = 90 # 降噪用

def audio_callback(indata, frames, time1, status):
    """音频输入回调函数"""
    indata[np.abs(indata) < INPUT_THRESHOLD] = 0

    # 将numpy数组转换为字节数据
    global is_awake
    global wake_time

    data_bytes = indata.tobytes()

    if is_awake and time.time() - wake_time > WAKE_TIMEOUT:
        is_awake = False
        logger.info("超时未说话")

    # 计算每帧的字节大小（样本数 * 通道数 * 样本宽度）
    frame_bytes = vad.frame_size * CHANNELS * 2  # 2 bytes per sample

    # 分割为单独的帧进行处理
    for i in range(0, len(data_bytes), frame_bytes):
        frame = data_bytes[i:i + frame_bytes]
        if len(frame) == frame_bytes:
            filename = vad.process_frame(frame)
            if filename:
                text = ASR2(filename, "wav", 16000)
                try:
                    os.remove(filename)
                except Exception as ex:
                    logger.warning(f"删除语音文件异常",exc_info = True)

                # 已经唤醒
                if is_awake:
                    is_awake = False
                    dct = {
                        "msg": text,
                        "status": get_status()
                    }
                    re = nlp(dct)
                # 未唤醒
                else:
                    re = awake(text)
                    if 'wake' in re and re['wake']:
                        is_awake = True
                        wake_time = time.time()
                        re['talk'] = "我在"

                if 'error' in re:
                    TTS(re['error'])
                if 'talk' in re:
                    TTS(re['talk'])
                if 'do' in re:
                    do_task(re['do'])
def get_status():
    # 输入status：0->待机, 1->执行任务中, 2->任务暂停中, 3->返航中, 4->地图初始化中， 5->地图未初始化
    try:
        get_statu = rospy.ServiceProxy('getStatu', statu)
        resp = get_statu(1)
        logger.debug(f"当前状态: {resp.status}")
        return resp.status
    except rospy.ServiceException as e:
        logger.error(f"请求service异常: {e}") 
        return None
    


def do_task(t):
    # 输出do：1->暂停任务, 2->继续任务, 3->开始拾取, 4->返回基站, 5->取消任务, 6->开始建图
    try:
        add_task = rospy.ServiceProxy('addTask', task)
        resp = add_task(t)
        pass
    except rospy.ServiceException as e:
        logger.error(f"请求service异常: {e}")
    logger.info(f"读取到状态: {task}")

    return 0

    


if __name__ == "__main__":

    logger = MyLogger("voice_interface_node").getLogger()
    vad = VoiceActivityDetector()

    logger.info("启动语音交互")
    rospy.set_param('/nodes_ready/interface_node', True)
    rospy.on_shutdown(lambda: rospy.delete_param('/nodes_ready/interface_node'))

    try:
        with sd.InputStream(
                callback=audio_callback,
                channels=CHANNELS,
                samplerate=SAMPLE_RATE,
                dtype=np.int16,
                blocksize=vad.frame_size
        ):
            while True:
                sd.sleep(1)
    except KeyboardInterrupt:
        logger.info("结束语音交互")
    except Exception as e:
        logger.exception("语音交互发生异常")
