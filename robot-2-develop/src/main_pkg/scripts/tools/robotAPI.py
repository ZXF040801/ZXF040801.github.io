
import logging
import rospy
from geometry_msgs.msg import Twist
import math

class RobotAPI():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    def stop(self):
        self.drive(0,0,0)
    
    def drive(self,x,y,z):
        m = Twist()
        m.linear.x = x
        m.linear.y = y
        m.angular.z = math.radians(z)
        self.pub.publish(m)
        
    
        
