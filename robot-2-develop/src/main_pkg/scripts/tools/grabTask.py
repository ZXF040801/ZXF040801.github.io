from tools.basic import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


import numpy as np

import time


camera_matrix = np.array([[2410.04743285676,	    0,	                1287.24883153473],
                          [0,	                2395.70419538681,	630.588345223324],
                          [0,	                0,	                1               ]], dtype=np.float32)

dist_coeffs = np.array([0.113869137599164,	-0.511084933645975, 0, 0, 0], dtype=np.float32)
T_gripper_cam = np.array([[  0.98241967,   0.13350184,  -0.13049467,  28.73110322],
                                      [ -0.14753449,   0.98351883,  -0.10451929, -16.59823054],
                                      [  0.11439045,   0.12193427,   0.98592437,  11.20634613],
                                      [  0.   ,        0. ,         0.   ,        1.        ]])

# end->cam

class GrabTask(Task):
    def __init__(self):
        self.log = logging.getLogger("mainNode.GrabTask")
        self.isFinish = False
        self.arm = ArmAPI()
        self.ispause = False
        self.ongrib = False
        self.exitflag = False
        self.t = time.time()
        self.goalType = None


    def init(self):
        self.log.info("Starting GrabTask")

    def work(self):
 

        if self.ispause:
            return

        
        track_msg = get_track_info()
        
        if self.ongrib:
            f = self.arm.isGrapMove()
            if not f:
                self.ongrib = False
                self.arm.waitArmMoveTo(ArmAPI.Pose.HIGH)
                self.log.info(f"{track_msg.type}   {type(track_msg.type)}")
                if self.goalType  == 1:
                    self.arm.waitArmMoveTo(ArmAPI.Pose.RIGHT)
                else:
                    self.arm.waitArmMoveTo(ArmAPI.Pose.LEFT)
                self.arm.waitGrib(1)
                self.arm.waitArmMoveTo(ArmAPI.Pose.HIGH)
                self.arm.waitArmMoveTo(ArmAPI.Pose.LOW)
                self.exitflag = True
                self.t = time.time()
            return

        if time.time() - self.t > 3:
            self.isFinish = True
            return
            

        ux = track_msg.ux
        uy = track_msg.uy
        h= track_msg.uh
        w= track_msg.uw
        if track_msg.id == -1:
            return
        
        self.goalType = track_msg.type
            
        self.t = time.time()
       
                
        if h>270 or w>270:
            self.log.info("Grib!")
            ang0 = self.arm.getAng()    
            self.ongrib = True
            a1,a2,a3,a4,a5,a6 = ang0
            self.arm.waitArmMoveAng([a1,a2+8,a3,a4,a5+5,a6])
            self.arm.setGrap(0)
            return



        #T_gripper = self.arm.getT()

        
        """
        kx = 0.05
        ky = 0.05
        kz = 0.1


        dy = -(ux-320)
        dx = -(uy - 240)
        dz = -(h - 400)
        
        
        x,y,z,rx,ry,rz = T_gripper
        
        x+=kx*dx
        y+=ky*dy
        z-=kz*dz

        grasp_6d = [x,y,z,rx,ry,rz]
        """
        
        dy = -(ux-300)
        dx = -(uy - 340)
        dz = -(h - 400)
        
        kx1 = 0.005
        kx2 = 0.001
        kx3 = 0.05
        ky = 0.025
        kz1 = 0.005
        kz2 = 0.003
        kz3 = 0.001 
        
        ang0 = self.arm.getAng()
        a1,a2,a3,a4,a5,a6 = ang0
        
        a1+=ky*dy
        
        da2=kx1*dx + kz1*dz
        da3=kx2*dx + kz2*dz
        da5=kx3*dx - kz3*dz
        
        if da2>50:
           da2 = 5
        if da2<-50:
           da2 = -5
                
        if da3>50:
           da3 = 5
        if da3<-50:
           da3 = -5
           
        if da5>50:
           da5 = 5
        if da5<-50:
           da5 = -5
        
        a2+=da2
        a3+=da3
        a5+=da5
        
        if a5 < -95:
            a5 = -95
        #if a3 < -47:
        #    a3 = -47
        
        ang = [a1,a2,a3,a4,a5,a6]
        
        self.log.info("--------------------------------")
        self.log.info(f"{da2} {da3} {da5}")
        self.log.info(f"{ux} {uy} {h} {w}")
        self.log.info(f"{dx} {dy} {dz}")
        self.log.info(ang0)
        self.log.info(ang)
        
        #ppp(T_gripper,grasp_6d)
        
        if ang:
            #self.arm.armMove(grasp_6d) 
            self.arm.armMoveAngle(ang)
        
        #time.sleep(0.1)
        
        
       

    def pause(self):
        self.ispause = True
        self.arm.armStop()

    def un_pause(self):
        self.ispause = False

    def cancel(self):
        self.arm.armStop()

    def is_finish(self):
        if self.isFinish:
            self.arm.waitArmMoveTo(ArmAPI.Pose.LOW)
        return self.isFinish

    def next_tasks(self):
        self.log.info("Finishing GrabTask")
        return [NAVI_TASK]

    def get_id(self):
        return GRAB_TASK


def ppp(current_pose,target_pose):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    plot_pose_frame(ax, current_pose, 'cyan', 'Initial Pose', axis_length=50)
    plot_pose_frame(ax, target_pose, 'magenta', 'Target Pose', axis_length=50)

    all_x = [current_pose[0], target_pose[0]]
    all_y = [current_pose[1], target_pose[1]]
    all_z = [current_pose[2], target_pose[2]]

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('Robot Arm Pose Control Simulation (No Flange Roll)')

    padding = 60
    ax.set_xlim([min(all_x) - padding, max(all_x) + padding])
    ax.set_ylim([min(all_y) - padding, max(all_y) + padding])
    ax.set_zlim([min(all_z) - padding, max(all_z) + padding])

    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', label='Initial Pose', markerfacecolor='cyan', markersize=10),
        Line2D([0], [0], marker='o', color='w', label='Target Pose', markerfacecolor='magenta', markersize=10),
        Line2D([0], [0], color='red', lw=2, label='EE X-axis'),
        Line2D([0], [0], color='green', lw=2, label='EE Y-axis'),
        Line2D([0], [0], color='blue', lw=2, label='EE Z-axis')
    ]
    ax.legend(handles=legend_elements)

    ax.view_init(elev=20., azim=-65)
    plt.tight_layout()
    plt.show()
    
def plot_pose_frame(ax, pose, color, label, axis_length=50):
    x, y, z, rx, ry, rz = pose
    position = np.array([x, y, z])
    R = euler_zyx_deg_to_matrix(rx, ry, rz)

    ax.scatter(position[0], position[1], position[2], color=color, label=label, s=100, depthshade=True)

    x_axis = R[:, 0] * axis_length
    y_axis = R[:, 1] * axis_length
    z_axis = R[:, 2] * axis_length

    ax.quiver(position[0], position[1], position[2], x_axis[0], x_axis[1], x_axis[2], color='red', arrow_length_ratio=0.1)
    ax.quiver(position[0], position[1], position[2], y_axis[0], y_axis[1], y_axis[2], color='green', arrow_length_ratio=0.1)
    ax.quiver(position[0], position[1], position[2], z_axis[0], z_axis[1], z_axis[2], color='blue', arrow_length_ratio=0.1)


