#!/usr/bin/env python3.6
import rospy

import logging
import logging.handlers
import socketserver
import pickle
import struct
import datetime
import os
import socket

FILE_LEVEL = logging.DEBUG
CONSOLE_LEVEL = logging.DEBUG

class ReusableThreadingTCPServer(socketserver.ThreadingTCPServer):
    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        super().server_bind()

class LogRequestHandler(socketserver.StreamRequestHandler):
        
    def handle(self):
        while True:           
            raw_len = self.rfile.read(4)
            if not raw_len:
                break  
            data_len = struct.unpack('>L', raw_len)[0]        
            data = self.rfile.read(data_len)
            if not data:
                break           
            record_dict = pickle.loads(data) 
            record = logging.makeLogRecord(record_dict)
            self.server.logger.handle(record)


if __name__ == "__main__":
    rospy.init_node('log_sever_node')

    root_logger = logging.getLogger("LoggerSever")
    root_logger.setLevel(logging.DEBUG)

    formatter = logging.Formatter('%(asctime)s [%(name)s] [%(levelname)s] - %(message)s')

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S")
    apploghome = os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","..","..","log","run")
    filename=os.path.join(apploghome,timestamp+".log")
    os.makedirs(apploghome,exist_ok=True)

    file_handler = logging.FileHandler(filename)
    file_handler.setLevel(FILE_LEVEL)
    file_handler.setFormatter(formatter)
    root_logger.addHandler(file_handler)

    console_handler = logging.StreamHandler()
    console_handler.setLevel(CONSOLE_LEVEL)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    server = ReusableThreadingTCPServer(('0.0.0.0', 9605), LogRequestHandler)
    server.logger = root_logger
    server.daemon_threads = True

    root_logger.info("Log Sever Start")
    root_logger.info(f"Saved to: {timestamp+'.log'}")

    def shutdown():
        root_logger.info("Log Sever Stop")
        root_logger.info(f"Saved to: {timestamp+'.log'}")
        os._exit(1)
        

    rospy.on_shutdown(shutdown)
    server.serve_forever()


