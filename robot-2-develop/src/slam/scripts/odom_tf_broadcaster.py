#!/usr/bin/env python2
import rospy, tf, math
from nav_msgs.msg import Odometry
import tf.transformations as tft    

rot180 = tft.quaternion_from_euler(0, 0, math.pi)  

def cb(msg):
  

    br = tf.TransformBroadcaster()
    br.sendTransform(
        (msg.pose.pose.position.x,
         msg.pose.pose.position.y,
         0),
        (msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w),
        msg.header.stamp,
        "base_link",        # child
        "odom"              # parent
    )

if __name__ == "__main__":
    rospy.init_node("odom_tf_broadcaster")
    rospy.Subscriber("/odom", Odometry, cb, queue_size=10)
    rospy.spin()
