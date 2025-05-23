from tools.basic import *
from tools.setGlobalStatusTask import SetGlobalStatusTask
from tools.homewardTask import HomewardTask
from tools.grabTask import GrabTask
from tools.naviTask import NaviTask
from tools.slamTask import SLAMTask



class GlobalStatus:
    def __init__(self, max_capacity=10):
        self.status = []  # 用于存储状态的列表
        self.isPause = False  # 暂停标志
        self.max_capacity = max_capacity  # 状态列表的最大容量

    def set(self, s):
        if len(self.status) >= self.max_capacity:
            self.status.pop()
        self.status.insert(0, s)

    def set_from_task(self, task):
        if task == GRAB_TASK or task == NAVI_TASK:
            self.set(WORK_STATUS)
        elif task==HOOM_TASK:
            self.set(HOOM_STATUS)
        elif task==SLAM_TASK:
            self.set(SLAM_STATUS)


    def back(self):
        if self.status:
            self.status.pop(0)

    def set_pause(self, flag):
        self.isPause = flag

    def get_status(self):
        if self.status:
            return self.status[0]
        return None
    def is_pause(self):
        return self.isPause

def get_task(id):

    if id == SLAM_TASK:
        return SLAMTask()
    elif id == NAVI_TASK:
        return NaviTask()
    elif id == HOOM_TASK:
        return HomewardTask()
    elif id == GRAB_TASK:
        return GrabTask()
    elif id == SET_GLOBAL_STATUS_TASK:
        return SetGlobalStatusTask(STAND_BY_STATUS)
