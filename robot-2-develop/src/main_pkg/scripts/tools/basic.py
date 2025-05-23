import logging
import time
import rospy
from vision_pkg.msg import TrackInfo
from tools.task import Task
from tools.robotAPI import RobotAPI
from tools.armAPI import ArmAPI

# 任务ID
SLAM_TASK = 1
NAVI_TASK = 2
HOOM_TASK = 3
GRAB_TASK = 4
SET_GLOBAL_STATUS_TASK = 5
# 状态ID
STAND_BY_STATUS = 0
WORK_STATUS = 1
PAUSE_STATUS = 2
HOOM_STATUS = 3
SLAM_STATUS = 4
NO_MAP_STATUS = 5
# 中断ID
PAUSE_IPT = 1
UN_PAUSE_IPT = 2
CANCEL_IPT = 3
CHANGE_TASK_IPT = 4



track_msg = None
def track_info_callback(msg):
    global track_msg
    track_msg = msg
 
def get_track_info():
    return track_msg

rospy.Subscriber("/track_info", TrackInfo, track_info_callback, queue_size=1)  



