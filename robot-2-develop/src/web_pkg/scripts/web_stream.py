#!/usr/bin/env python
import rospy
import json
from sensor_msgs.msg import Image
from vision_pkg.msg import TrackInfo
from cv_bridge import CvBridge
import cv2
import threading
import time
from flask import Flask, Response, render_template, request, jsonify

app = Flask(__name__)
bridge = CvBridge()
current_frame = None
frame_lock = threading.Lock()


system_status = {
    "ros_connected": True,
    "fps": 0,
    "resolution": "640x480",
    "recording": False,
    "task_paused": False, 
    "track_info": {},  
    "last_update": time.time()
}
status_lock = threading.Lock()

def image_callback(msg):
    global current_frame
    try:
        start_time = time.time()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cv2.resize(cv_image, (640, 480))
        _, jpeg = cv2.imencode('.jpg', cv_image, [
            int(cv2.IMWRITE_JPEG_QUALITY), 10,
            int(cv2.IMWRITE_JPEG_OPTIMIZE), 1
        ])
        
        with frame_lock:
            current_frame = jpeg.tobytes()
        
     
        with status_lock:
            system_status["fps"] = 1 / (time.time() - system_status["last_update"])
            system_status["last_update"] = time.time()
    except Exception as e:
        rospy.logerr("Image processing error: %s" % str(e))


def track_info_callback(msg):
    try:
        with status_lock:
            system_status["track_info"] = {
                "id": msg.id,
                "type": msg.type
            }
    except Exception as e:
        rospy.logerr("Track info error: %s" % str(e))

def generate_frames():
    while True:
        with frame_lock:
            if current_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + current_frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/system_status')
def get_status():
    with status_lock:
        return jsonify(system_status)

@app.route('/control', methods=['POST'])
def control():
    command = request.json.get('command')
    response = {"status": "success"}
    
    if command == "start_recording":
        with status_lock:
            system_status["recording"] = True
    elif command == "stop_recording":
        with status_lock:
            system_status["recording"] = False
    elif command == "pause_task":
        with status_lock:
            system_status["task_paused"] = True
        print("Task paused")  
    elif command == "resume_task":
        with status_lock:
            system_status["task_paused"] = False
        print("Task resumed") 
    elif command == "snapshot":
        print("Snapshot taken") 
    else:
        response["status"] = "error"
        response["message"] = "Unknown command"
    
    return jsonify(response)

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    rospy.init_node('web_video_stream', anonymous=True)
    
   
    rospy.Subscriber("/vision/image_raw", Image, image_callback, queue_size=1)
    rospy.Subscriber("/track_info", TrackInfo, track_info_callback, queue_size=1)
    
   
    flask_thread = threading.Thread(
        target=lambda: app.run(
            host='0.0.0.0',
            port=5000,
            threaded=True,
            debug=True,
            use_reloader=False
        )
    )
    flask_thread.daemon = True
    flask_thread.start()
    
    rospy.spin()
