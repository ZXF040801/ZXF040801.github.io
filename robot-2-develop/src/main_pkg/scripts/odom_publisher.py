#!/usr/bin/env python3
import rospy
from math import radians
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3, Pose
from tools.robotAPI import RobotAPI 
import math
import sys
sys.path.insert(0,'/home/jetson/robot-2/src/log_pkg/scripts')
from log import MyLogger


def publish_odom(event):

    position = robot.position
    attitude = robot.attitude
    velocity = robot.velocity
    imu = robot.imu
    if (not position) or (not attitude) or (not velocity) or (not imu):
        log.warn("data not ready")
        return

    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    yaw = -radians(attitude[0])
    quaternion = [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]

    odom.pose.pose = Pose(
        Point(position[0], -position[1], 0),
        Quaternion(*quaternion)
    )

    odom.twist.twist = Twist(
        Vector3(velocity[0], velocity[1], 0),
        Vector3(0, 0, radians(imu[5]))
    )

    odom_pub.publish(odom)


try:
    log = MyLogger("odomPublisher").getLogger()
    rospy.init_node('odom_publisher_node')
    robot = RobotAPI()
    if not robot:
        log.error("robot api error")
        exit(-1)

    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
    
    log.info("odom pubisher node ready")

    rospy.Timer(rospy.Duration(0.01), publish_odom)

except rospy.ROSInterruptException:
    pass


