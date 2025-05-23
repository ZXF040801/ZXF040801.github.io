#!/usr/bin/env python3
import rospy
import curses
import time
from geometry_msgs.msg import Twist
import math

rospy.init_node("car_drive")

def send(vx,vy,vz):
    m = Twist()
    m.linear.x = vx
    m.linear.y = vy
    m.angular.z = math.radians(vz)
    pub.publish(m)

def main(stdscr):
    stdscr.nodelay(True)
    vx = 0
    vy = 0
    vz = 0 
    
    while True:
        key = stdscr.getch()
        if key != -1:
            print(key)
        if key == 27: # ESC
            print("ESC")  
            #r.chassis.drive_speed(0,0,0)
            send(0,0,0)
            break
        if key==32:
            vx = 0
            vy = 0
            vz = 0 
        
        if key == 119: # w
            vx = 0.15
        if key == 115:
            vx = -0.15
        if key == 97:
            vy = -0.15
        if key==100:
            vy= 0.15
        if key==113:
            vz= -20
        if key==101:
            vz= 20
        #r.chassis.drive_speed(vx,vy,vz)
        send(vx,-vy,-vz)
        time.sleep(0.01)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
curses.wrapper(main)
