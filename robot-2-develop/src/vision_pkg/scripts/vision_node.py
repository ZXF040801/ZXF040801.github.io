#!/usr/bin/env python3
import sys

sys.path.insert(0, '/home/jetson/robot-2/src/log_pkg/scripts')
from log import MyLogger

import rospy
from sensor_msgs.msg import Image
from vision_pkg.msg import TrackInfo
from main_pkg.srv import visionStart, visionStartResponse

from basicModel import YoLov5TRT
from sort import *
import cv2
import os
import time


class Track(YoLov5TRT):
    def __init__(self, engine_path, plugin_file_path, max_age=10):
        super(Track, self).__init__(engine_path, plugin_file_path)
        self.max_age = max_age
        self.trackers = {}

    def track(self, frame):
        """
            Parameters:
                img: ndarray

           Returns:
                list: [[name,idx,bbox]]
        """

        detections = {}
        boxes, scores, ids = self.infer([frame])[0]
        bboxs = []
        for j in range(len(boxes)):
            name = ids[j]
            bbox = boxes[j]
            score = scores[j]
            if not name in self.trackers.keys():
                self.trackers[name] = Sort(max_age=self.max_age)
            if not name in detections.keys():
                detections[name] = []
            detections[name].append([bbox[0], bbox[1], bbox[2], bbox[3], score])
        for name in self.trackers.keys():
            if name not in detections.keys():
                self.trackers[name].update()
                continue
            track_bbs_ids = self.trackers[name].update(np.array(detections[name]))
            for track in track_bbs_ids:
                bboxs.append([name, track[4], track[:4]])
        return bboxs


def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height  # That double line is actually integer division, not a comment
    return img_msg


def handle_service(req):
    global startFlag
    if req.start:
        startFlag = True
    else:
        startFlag = False
    return visionStartResponse(1)


if __name__ == "__main__":

    log = MyLogger("visionNode").getLogger()
    rospy.init_node('vision_node')

    pub = rospy.Publisher('track_info', TrackInfo, queue_size=1)

    current_dir = os.path.dirname(os.path.abspath(__file__))
    plugin_file_path = os.path.join(current_dir, "source/libmyplugins.so")
    engine_file_path = os.path.join(current_dir, "source/ppq-5.engine")

    yolov5_tracker = Track(engine_file_path, plugin_file_path, max_age=10)
    cap = cv2.VideoCapture(0)
    # cap = cv2.VideoCapture(os.path.join(current_dir,"source/TEST.mp4"))
    # cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # cap.set(cv2.CAP_PROP_FPS, 30)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    show = False
    pub_img = True
    fps = 0
    if show or pub_img:
        t_start = time.time()
        frame_cnt = 0

    if pub_img:
        img_pub = rospy.Publisher("/vision/image_raw", Image, queue_size=1)

    goalType = None
    goalId = None
    lossTime = time.time() - 1

    msg = TrackInfo()
    startFlag = True
    s = rospy.Service('setVisionStart', visionStart, handle_service)

    rospy.set_param('/nodes_ready/vision_node', True)
    rospy.on_shutdown(lambda: rospy.delete_param('/nodes_ready/vision_node'))
    log.debug("vision_node is ready")

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if not ret:
            break

        if not startFlag:
            if show:
                cv2.imshow("1", frame)
                cv2.waitKey(1)
            if pub_img:
                ros_image = cv2_to_imgmsg(frame)
                ros_image.header.stamp = rospy.Time.now()
                img_pub.publish(ros_image)
            continue

        bboxs = yolov5_tracker.track(frame)

        if len(bboxs) == 0:
            msg.id = -1
            msg.type = -1
            msg.ux = -1
            msg.uy = -1
            msg.uw = -1
            msg.uh = -1
        else:
            maxw = 0
            maxd = None
            for d in bboxs:
                name, idx_id, box = d
                x1, y1, x2, y2 = box
                getGoal = False

                if int(name) == goalType and int(idx_id) == goalId:
                    lossTime = time.time()
                    msg.id = int(idx_id)
                    msg.type = int(name)
                    msg.ux = (x2 + x1) / 2
                    msg.uy = (y2 + y1) / 2
                    msg.uw = (x2 - x1)
                    msg.uh = (y2 - y1)
                    if show or pub_img:
                        label = f"{int(name)} id: {int(idx_id)}"
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                        cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0),
                                    1)

                    getGoal = True
                else:
                    if show or pub_img:
                        label = f"{int(name)} id: {int(idx_id)}"
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0),
                                    1)

                if x2 - x1 > maxw:
                    maxw = x2 - x1
                    maxd = d.copy()

            if not getGoal and time.time() - lossTime > 0.1:
                name, idx_id, box = maxd
                x1, y1, x2, y2 = box
                msg.id = int(idx_id)
                msg.type = int(name)
                #msg.type = 1
                msg.ux = (x2 + x1) / 2
                msg.uy = (y2 + y1) / 2
                msg.uw = (x2 - x1)
                msg.uh = (y2 - y1)
                goalId = int(idx_id)
                goalType = int(name)

        if show or pub_img:
            frame_cnt += 1
            if time.time() - t_start > 1:
                fps = frame_cnt / (time.time() - t_start)
                frame_cnt = 0
                t_start = time.time()
            cv2.putText(frame, f"{fps:.2f} fps", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        if show:
            cv2.imshow("1", frame)
            cv2.waitKey(1)
        if pub_img:
            ros_image = cv2_to_imgmsg(frame)
            ros_image.header.stamp = rospy.Time.now()
            img_pub.publish(ros_image)

        pub.publish(msg)



