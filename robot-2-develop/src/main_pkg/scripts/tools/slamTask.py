from tools.basic import *
from main_pkg.srv import visionStart, visionStartResponse

class SLAMTask(Task):
    def __init__(self):
        self.log = logging.getLogger("mainNode.SLAMTask")
        self.t = time.time()
        setStart = rospy.ServiceProxy('setVisionStart', visionStart)
        resp = setStart(0)

    def init(self):
        self.log.info("Starting SLAMTask")

    def work(self):
        pass

    def pause(self):
        self.log.debug("Pausing SLAMTask")

    def un_pause(self):
        self.log.debug("Un-pausing SLAMTask")

    def cancel(self):
        self.log.debug("Cancelling SLAMTask")

    def is_finish(self):
        return time.time() - self.t > 5

    def next_tasks(self):
        self.log.info("Finishing SLAMTask")
        return [SET_GLOBAL_STATUS_TASK, HOOM_TASK]

    def get_id(self):
        return SLAM_TASK
