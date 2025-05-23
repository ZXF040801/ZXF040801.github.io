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
WAKE_TIMEOUT = 600



def get_status():
    try:
        get_statu = rospy.ServiceProxy('getStatu', statu)
        resp = get_statu(1)
        logger.debug(f"status: {resp.status}")
        return resp.status
    except rospy.ServiceException as e:
        logger.error(f"service error: {e}") 
        return None
    


def do_task(t):
    try:
        add_task = rospy.ServiceProxy('addTask', task)
        resp = add_task(t)
        pass
    except rospy.ServiceException as e:
        logger.error(f"service error: {e}")
    logger.info(f"do task: {task}")

    return 0

    


if __name__ == "__main__":

    logger = MyLogger("voice_interface_node").getLogger()
    

    logger.info("keyboard node ready")
    rospy.set_param('/nodes_ready/interface_node', True)
    rospy.on_shutdown(lambda: rospy.delete_param('/nodes_ready/interface_node'))

    while True:
        print("input: ")
        text = input()
        logger.debug(f"read: {text}")

        if is_awake:
            is_awake = False
            dct = {
                "msg": text,
                "status": get_status()
                }
            re = nlp(dct)
        
        else:
            re = awake(text)
            
        if 'wake' in re and re['wake']:
            is_awake = True
            wake_time = time.time()
            re['talk'] = "wo zai"
  
        if 'error' in re:
            TTS(re['error'])
        if 'talk' in re:
            TTS(re['talk'])
        if 'do' in re:
            do_task(re['do'])
    
    
    