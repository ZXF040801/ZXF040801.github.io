#!/usr/bin/env python3
import sys
import time

import rospy

sys.path.insert(0,'/home/jetson/robot-2/src/log_pkg/scripts')
from log import MyLogger

#from src.scripts.log import * # 仅导入一次
from tools.tools import *
from main_pkg.srv import task, taskResponse
from main_pkg.srv import statu, statuResponse

def check_map():
    return True

def wait_all_node(log):
    # TODO: 更新node列表
    expected_nodes = ["arm_node_pi","vision_node"]
    # 检查频率（Hz）
    check_rate = rospy.Rate(0.5)
    start_time = time.time()
    while True:
        # 超时检测 1分钟
        if time.time() - start_time > 60:
            log.warning("Time out")
            start_time = time.time()

        # 检查所有节点就绪状态
        copy_expected_node = expected_nodes.copy()
        for node in copy_expected_node:
            param_name = '/nodes_ready/{}'.format(node)
            if rospy.has_param(param_name) and rospy.get_param(param_name):
                log.info(f"node {node} is ready.")
                expected_nodes.remove(node)

        if len(expected_nodes) == 0:
            log.info(f"all nodes are ready.")
            return

        if rospy.is_shutdown():
           exit(1)
        check_rate.sleep()
        
def handle_task(req):
    # 输出do：1->暂停任务, 2->继续任务, 3->开始拾取, 4->返回基站, 5->取消任务, 6->开始建图
    global tasks
    global log
    log.info(f"添加任务: {req.num}")
    if req.num == 1:
        interrupts.append(PAUSE_IPT)
    elif req.num == 2:
        interrupts.append(UN_PAUSE_IPT)
    elif req.num == 3:
        tasks.append(NAVI_TASK)
    elif req.num == 4:
        interrupts.append(CHANGE_TASK_IPT)
        tasks = [HOOM_TASK]
    elif req.num == 5:
        interrupts.append(CANCEL_IPT)
    elif req.num == 6:
        interrupts.append(CHANGE_TASK_IPT)
        tasks.append(SLAM_TASK)
    return taskResponse(1)

def handle_statu(req):
    global status
    if status.is_pause():
        return statuResponse(PAUSE_STATUS)
    else:
        return statuResponse(status.get_status())

if __name__ == '__main__':
    rospy.init_node('main_node')


    log = MyLogger("mainNode").getLogger()

    s1 = rospy.Service('addTask', task, handle_task)
    s2 = rospy.Service('getStatu', statu, handle_statu)

    status = GlobalStatus()
    tasks = []
    interrupts = []
    task_now:Task = None

    log.info("main node init, wait all node ready...")
    wait_all_node(log)
    log.info("----main node start----")
    
    log.info("init...")
    
    arm = ArmAPI()
    #arm.waitArmMoveTo(ArmAPI.Pose.LOW)
    
    log.info("System Ready!")

    if check_map():
        status.set(STAND_BY_STATUS)
    else:
        status.set(NO_MAP_STATUS)

    while not rospy.is_shutdown():
        if task_now:
            task_now.work()
        if tasks and task_now is None:
            task_id = tasks.pop(0)
            log.info(f"{task_id}   {tasks}")
            task_now = get_task(task_id)
            task_now.init()
            status.set_from_task(task_id)
        if interrupts:
            ipt = interrupts.pop(0)
            if ipt == PAUSE_IPT:
                log.info(f"PAUSE")
                task_now.pause()
                status.set_pause(True)
            elif ipt == UN_PAUSE_IPT:
                log.info(f"UN_PAUSE")
                task_now.un_pause()
                status.set_pause(False)
            elif ipt == CANCEL_IPT:
                log.info(f"CANSEL")
                task_now.cancel()
                task_now = None
                tasks = []
                status.back()
            elif ipt == CHANGE_TASK_IPT:
                log.info(f"CHANGE_TASK")
                task_now = None
        if task_now and task_now.is_finish():
            next_task = task_now.next_tasks()
            if next_task:
                tasks = next_task + tasks
            task_now = None

