from abc import ABC, abstractmethod

class Task(ABC):

    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def work(self):
        pass

    @abstractmethod
    def pause(self):
        pass

    @abstractmethod
    def un_pause(self):
        pass

    @abstractmethod
    def cancel(self):
        pass

    @abstractmethod
    def is_finish(self):
        pass

    @abstractmethod
    def next_tasks(self):
        pass

    @abstractmethod
    def get_id(self):
        pass