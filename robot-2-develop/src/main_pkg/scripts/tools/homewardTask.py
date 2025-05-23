from tools.basic import *

class HomewardTask(Task):
    def __init__(self):
        self.log = logging.getLogger("mainNode.HomewardTask")
        self.t = time.time()

    def init(self):
        self.log.info("Starting HomewardTask")

    def work(self):
        pass

    def pause(self):
        pass

    def un_pause(self):
        pass

    def cancel(self):
        pass

    def is_finish(self):
        return time.time() - self.t > 5

    def next_tasks(self):
        self.log.info("Finishing HomewardTask")
        return [SET_GLOBAL_STATUS_TASK]

    def get_id(self):
        return HOOM_TASK