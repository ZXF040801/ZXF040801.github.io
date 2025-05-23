from tools.basic import *
from main_pkg.srv import visionStart, visionStartResponse
import rospy
from vision_pkg.msg import TrackInfo
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import time
import os
import json

current_dir = os.path.dirname(os.path.abspath(__file__))
WAYPOINT_FILE= "/home/jetson/robot-2/src/navigation/scripts/waypoints.npy"

INIT_POSE = (0.2, 0.0)
GOAL_FRAME_ID = "map"

TEMP_FILE = os.path.join(current_dir, "navi_temp.json")

class NaviTask(Task):
    def __init__(self):
        self.log = logging.getLogger("mainNode.NaviTask")
        #setStart = rospy.ServiceProxy('setVisionStart', visionStart)
        #resp = setStart(1)
        self.ispause = False     
        self.isfinish = False
        self.havetask = False    
        self.waypoints = None
        self.robot = RobotAPI()
        
        try:
            self.waypoints = np.load(WAYPOINT_FILE)
            self.log.info(f"load {len(self.waypoints)} waypoints.")
        except Exception as e:
            self.log.error(f"waypoint file error: {e}")
            return
            
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        self.log.info("wait for move_base client...")
        if not self.client.wait_for_server(rospy.Duration(60.0)):
            self.log.error("Time out waiting for move_base server")
            return
            
        self.log.info("connect to move_base server")
        self.current_waypoint_index = 0
        
        self.t = time.time()
        
        # check temp
        idx = self.__read_temp()
        if idx != -1:
            self.current_waypoint_index = idx


    def init(self):
        self.log.info("Starting NaviTask")
        

    def __read_temp(self):
        with open(TEMP_FILE, "r") as f:
            data = json.load(f)
            return data['id']
    
    
    def __save_temp(self):
        with open(TEMP_FILE, "w") as f:
            json.dump({"id":self.current_waypoint_index},f)
    
    
# ing: 0 1
# err: 4 5 
# calcel:2 7
# suss 3


    def work(self):
    
        if self.ispause:
            return
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.isfinish = True
            self.log.info(f"navi finish")
            self.current_waypoint_index = -1
            self.__save_temp()
            return
    
        if not self.havetask:
            self.send_goal()
            self.havetask = True
        
        
        status = self.client.get_state() 
        
        
        if status == 0 or status == 1:
            # check track info
            track_msg = get_track_info()
            ux = track_msg.ux
            uy = track_msg.uy
            
            
            
            if track_msg.id != -1 and 128<ux<512:
                self.log.debug(f"track: {track_msg.id} {ux}")
                self.cancel()
                
                self.isfinish = True
                return
                
        
        elif status == 4 or status == 5:
            self.log.warn(f"mobe_base error: retry...")
            time.sleep(1)
            self.send_goal()
            
        elif status == 3:
            self.log.info(f"goal success")
            self.current_waypoint_index+=1
            self.havetask = False
              
            

    def pause(self):
        self.ispause = True
        if self.haveTask: 
            self.client.cancel_goal()
        self.haveTask = False


    def un_pause(self):
        self.ispause = False


    def cancel(self):
        self.robot.stop() 
        
        if self.havetask:
            self.client.cancel_goal()
            self.log.info("wait for goal cancel...")
            #while True:
            #    status = self.client.get_state() 
            #    if status == 2 or status == 7:
            #        break 
            time.sleep(1)
        self.__save_temp()
                
        self.isfinish = True 
        self.haveTask = False
        
            
        

    def is_finish(self):
        if self.isfinish:
            self.robot.stop()
            
        return self.isfinish

    def next_tasks(self):
        return [SET_GLOBAL_STATUS_TASK, GRAB_TASK]


    def get_id(self):
        return "NAVI_TASK"

   
                
    def __create_move_base_goal(self, x, y, yaw):
       goal = MoveBaseGoal()
       goal.target_pose.header.frame_id = "map"
       goal.target_pose.header.stamp = rospy.Time.now()
       goal.target_pose.pose.position = Point(x, y, 0.0)
       
  
       q = Quaternion()
       q.x = 0.0
       q.y = 0.0
       q.z = math.sin(yaw * 0.5)
       q.w = math.cos(yaw * 0.5)
       goal.target_pose.pose.orientation = q
       
       return goal
       
       
    def __calculate_yaw(self,p1_x, p1_y, p2_x, p2_y):
        return math.atan2(p2_y - p1_y, p2_x - p1_x)
    
       
    def send_goal(self):
        goalID = self.current_waypoint_index   
        
        x,y = self.waypoints[goalID]
        
        if goalID < len(self.waypoints) - 1:
            next_x, next_y = self.waypoints[goalID+1]
            yaw = self.__calculate_yaw(x, y, next_x, next_y)
        elif goalID > 0:
            prev_x, prev_y = self.waypoints[goalID-1]
            yaw = self.__calculate_yaw(prev_x, prev_y, x, y)
        else:
            yaw = 0.0
        
        self.log.info(f"send goal {goalID+1}/{len(self.waypoints)}: (x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f})")
        
        goal = self.__create_move_base_goal(x,y,yaw)
        
        self.client.send_goal(goal)
        

       
    def ddd(self):
              i = self.current_waypoint_index
              self.current_waypoint_index += 1
      
              waypoints = self.waypoints
              client = self.client 
              
              
              current_wp_x = waypoints[i][0]
              current_wp_y = waypoints[i][1]
              
              # Determine orientation
              if i < len(waypoints) - 1: # If there is a next waypoint
                  next_wp_x = waypoints[i+1][0]
                  next_wp_y = waypoints[i+1][1]
                  yaw = calculate_yaw(current_wp_x, current_wp_y, next_wp_x, next_wp_y)
              elif i > 0: # For the last waypoint, use orientation from previous segment
                  prev_wp_x = waypoints[i-1][0]
                  prev_wp_y = waypoints[i-1][1]
                  yaw = calculate_yaw(prev_wp_x, prev_wp_y, current_wp_x, current_wp_y)
              else: # Single waypoint, default orientation (0 yaw)
                  yaw = 0.0
                  
              rospy.loginfo(f"Sending goal {i+1}/{len(waypoints)}: (x={current_wp_x:.2f}, y={current_wp_y:.2f}, yaw={math.degrees(yaw):.2f} deg)")
              goal = create_move_base_goal(current_wp_x, current_wp_y, yaw)
              
              client.send_goal(goal)
              
              # Wait for the goal to complete or timeout
              wait_duration = rospy.Duration(120.0) # Example: 2 minutes per waypoint
              finished_within_time = client.wait_for_result(wait_duration) 
      
              if not finished_within_time:
                  client.cancel_goal()
                  rospy.logwarn(f"Goal {i+1} timed out. Cancelling and moving to next if available.")
              else:
                  state = client.get_state()
                  if state == actionlib.GoalStatus.SUCCEEDED:
                      rospy.loginfo(f"Goal {i+1} succeeded!")
                  elif state == actionlib.GoalStatus.PREEMPTED:
                      rospy.loginfo(f"Goal {i+1} was preempted.")
                      # Decide if you want to stop or continue on preemption
                      return 
                  elif state == actionlib.GoalStatus.ABORTED:
                      rospy.logwarn(f"Goal {i+1} aborted by move_base.")
                      # Optionally, you might want to retry or skip
                  else:
                      rospy.logwarn(f"Goal {i+1} finished with state: {client.get_goal_status_text()} (State code: {state})")
       
                
       
       
def euler_to_quaternion_yaw_only(yaw):
    """
    Converts a yaw angle (in radians) to a quaternion (x, y, z, w).
    Assumes roll and pitch are 0.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qz = sy
    qw = cy
    return Quaternion(0, 0, qz, qw) # geometry_msgs.msg.Quaternion

def create_move_base_goal(x, y, yaw):
    """Creates a MoveBaseGoal message."""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = GOAL_FRAME_ID
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = Point(x, y, 0.0)
    
    goal.target_pose.pose.orientation = euler_to_quaternion_yaw_only(yaw)
    
    return goal

def calculate_yaw(p1_x, p1_y, p2_x, p2_y):
    return math.atan2(p2_y - p1_y, p2_x - p1_x)
