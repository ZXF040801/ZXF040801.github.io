#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher', anonymous=True)
        
        self.device_id = rospy.get_param('~device_id', 0)
        self.frame_width = rospy.get_param('~width', 640)
        self.frame_height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 30)
        self.topic_name = rospy.get_param('~topic', '/vision/image_raw')
        
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera device /dev/video{}".format(self.device_id))
            exit(1)
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(self.topic_name, Image, queue_size=1)
        
        self.warn_interval = 1.0
        self.last_warn_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            
            if not ret:
                now = rospy.Time.now()
                if (now - self.last_warn_time).to_sec() > self.warn_interval:
                    rospy.logwarn("Camera frame read failed")
                    self.last_warn_time = now
                continue
                
            try:
                cv2.imshow("1",frame)
                cv2.waitKey(1)
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = rospy.Time.now()
                self.publisher.publish(ros_image)
            except Exception as e:
                rospy.logerr("Image conversion failed: {}".format(str(e)))
            
            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        publisher = CameraPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
