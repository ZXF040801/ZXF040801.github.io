import logging
import logging.handlers

class MyLogger:
    def __init__(self,appname):
        self.setupLogging2(appname=appname)

    def setupLogging2(self, appname, level=logging.DEBUG):
        logger = logging.getLogger(appname)
        logger.setLevel(level=logging.DEBUG)
	    
        socket_handler = logging.handlers.SocketHandler('localhost',9605)
        logger.addHandler(socket_handler)

        self.log = logger
    

    def getLogger(self):
        return self.log
