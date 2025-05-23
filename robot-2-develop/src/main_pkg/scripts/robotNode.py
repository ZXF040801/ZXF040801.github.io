#!/usr/bin/env python3
import sys
import time
import math
from robomaster import robot


sys.path.insert(0,'/home/jetson/robot-2/src/log_pkg/scripts')
from log import MyLogger

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3, Pose
import rospy

class RobotNode:

    def __init__(self):
        self.log = log = MyLogger("RobotAPI").getLogger()
        self.robot = robot.Robot()
        self.position = None
        self.velocity = None
        self.attitude = None
        self.imu = None
        # 等待连接成功
        self.__wait_for_connect()
        self.chassis:Chassis = self.robot.chassis
        self.led:Led = self.robot.led
        self.__sub_info()
        self.ctl_sub = rospy.Subscriber('/cmd_vel', Twist,lambda msg: self.__callback(msg), queue_size=1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        rospy.Timer(rospy.Duration(0.01), lambda e: self.__publish_odom(e))



    def __sub_info(self):
        # 订阅位置, IMU, 姿态
        def position_callback(data):
            # x y z
            self.position = data

        def attitude_callback(data):
            #yaw, pitch, roll
            self.attitude = data

        def imu_callback(data):
            self.imu = data

        def velocity_callback(data):
            # 上电时刻 vgx, vgy, vgz, 当前时刻 vbx, vby, vbz
            self.velocity = data[3:]

        # freq – enum: (1, 5, 10, 20, 50)
        freq = 5
        if not (self.chassis.sub_position(cs=0, freq=freq, callback=position_callback) and
                        self.chassis.sub_imu(freq=freq, callback=imu_callback) and
                        self.chassis.sub_velocity(freq=freq, callback=velocity_callback) and
                        self.chassis.sub_attitude(freq=freq, callback=attitude_callback)):
            self.log.error("sub information error!")
            raise Exception("sub information error")


    def __wait_for_connect(self):
        self.log.info("Waiting for connection...")
        # conn_type – 连接建立类型: ap表示使用热点直连；sta表示使用组网连接，rndis表示使用USB连接
        # proto_type – 通讯方式: tcp, udp
        try_cnt = 0
        while True:
            try:    
                re = self.robot.initialize(conn_type='rndis')
                if re:
                    self.log.info(f"Connection established! sn: {self.robot.get_sn()}")
                    break
                else:
                    if try_cnt > 5:
                        self.log.error("try max times!")
                        raise Exception("Connection timed out")
                self.log.info(f"Try {try_cnt}: Connection failed, 5 second later try again")
                time.sleep(5)
            except Exception as e:
                try_cnt += 1
                
    def set_speed(self,x,y,z):
        self.chassis.drive_speed(x,y,z)
        
    def __callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = math.degrees(msg.angular.z)
        self.set_speed(vx,-vy,-vz)

    def __publish_odom(self, event):

        position = self.position
        attitude = self.attitude
        velocity = self.velocity
        imu = self.imu
        if (not position) or (not attitude) or (not velocity) or (not imu):
            self.log.warn("data not ready")
            return

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        yaw = -math.radians(attitude[0])
        quaternion = [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]

        odom.pose.pose = Pose(
            Point(position[0], -position[1], 0),
            Quaternion(*quaternion)
        )

        odom.twist.twist = Twist(
            Vector3(velocity[0], velocity[1], 0),
            Vector3(0, 0, math.radians(imu[5]))
        )

        self.odom_pub.publish(odom)
        

if __name__ == "__main__":
    rospy.init_node("RobotNode node")
    r = RobotNode()
    rospy.spin()

