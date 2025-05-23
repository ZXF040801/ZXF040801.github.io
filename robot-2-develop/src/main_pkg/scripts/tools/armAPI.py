import logging
from enum import Enum
import rospy
from main_pkg.srv import armCtl, armCtlRequest
import time

ARM_MOVE = 1
ARM_STOP = 2
SET_GRAP = 3

IS_MOVE = 4
IS_GRAP_MOVE = 5
GET_T = 6
GET_ANG = 8

ARM_MOVE_ANG = 7

class ArmAPI:
    class Pose(Enum):
        LEFT = 0
        RIGHT = 1
        HIGH = 3
        LOW = 4

    def __init__(self):
        self.log = logging.getLogger("mainNode.ArmAPI")
        self.cmd = rospy.ServiceProxy('armCtl', armCtl)

    def armMove(self, pos):
        req = armCtlRequest()
        req.cmd = ARM_MOVE
        req.pose = pos
        self.cmd(req)
    
    def armMoveAngle(self, ang):
        req = armCtlRequest()
        req.cmd = ARM_MOVE_ANG
        #a1,a2,a3,a4,a5,a6 = ang
        #req.pose = [a1,a2,a3,a4,-a5,a6]
        req.pose = ang
        self.cmd(req)
    
    
    def armStop(self):
        req = armCtlRequest()
        req.cmd = ARM_STOP
        self.cmd(req)

    def armMoveTo(self, pos: Pose):
        if pos == self.Pose.LEFT:
            self.armMoveAngle([104,5.74,-38,0,-89,18])
        if pos == self.Pose.RIGHT:
            self.armMoveAngle([-104,5.74,-38,0,-89,18])
        if pos == self.Pose.HIGH:
            self.armMoveAngle([0,5.74,-38,0,-89,18])    
        if pos == self.Pose.LOW:
            self.armMoveAngle([0,11.33,-31.16,0,-90.43,18.28])       

    def setGrap(self, open: bool):
        req = armCtlRequest()
        req.cmd = SET_GRAP
        req.isGrapOpen = open
        self.cmd(req)

    def isMove(self):
        req = armCtlRequest()
        req.cmd = IS_MOVE
        resp = self.cmd(req)
        return resp.flag

    def isGrapMove(self):
        req = armCtlRequest()
        req.cmd = IS_GRAP_MOVE
        resp = self.cmd(req)
        return resp.flag

    def getT(self):
        req = armCtlRequest()
        req.cmd = GET_T
        resp = self.cmd(req)
        return resp.T
        
    def getAng(self):
        req = armCtlRequest()
        req.cmd = GET_ANG
        resp = self.cmd(req)
        #a1,a2,a3,a4,a5,a6 = resp.T
        #return [a1,a2,a3,a4,-a5,a6]
        return resp.T
 
    def waitArmMoveTo(self, pos: Pose):
        self.armMoveTo(pos)
        time.sleep(1)
        while True:
            if not self.isMove():
                break
            time.sleep(1)
    
    def waitGrib(self,open):
        self.setGrap(open)
        time.sleep(1)
        while True:
            if not self.isGrapMove():
                break
            time.sleep(0.2)
    
    def waitArmMoveAng(self, ang):
        self.armMoveAngle(ang)
        time.sleep(1)
        while True:
            if not self.isMove():
                break
            time.sleep(1)

if __name__ == "__main__":
    a = ArmAPI()
    print("start")
    
    a.setGrap(1)
    
