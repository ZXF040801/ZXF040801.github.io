from tools.basic import *

class SetGlobalStatusTask(Task):
    def __init__(self, id):
        self.id = id

    def init(self):
        pass

    def work(self):
        pass

    def pause(self):
        pass

    def un_pause(self):
        pass

    def cancel(self):
        pass

    def is_finish(self):
        return True

    def next_tasks(self):
        return None

    def get_id(self):
        return self.id