#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
import numpy as np
import matplotlib.pyplot as plt

global strt, t_init, t_now
strt = True
t_init = 0.0
t_now = 0.0

global X, Y, th, V, W
X = 0.0
Y = 0.0
th = 0.0
V = 0.0
W = 0.0

def limit(x,x_min,x_max):
    if(x<x_min):
        return x_min
    elif(x>x_max):
        return x_max
    else:
        return x

def wrap_pi(b_):
    if (b_ <= -math.pi):
        while(b_ <= -math.pi):
            b_ = b_ + 2.0*math.pi
    elif (b_ > math.pi):
        while(b_ > math.pi):
            b_ = b_ - 2.0*math.pi
    #print(b_*180.0/math.pi)
    return b_

def wrap_pi_2(b_):
    if (b_ <= -math.pi/2.0):
        while(b_ <= -math.pi/2.0):
            b_ = b_ + math.pi
    elif (b_ > math.pi/2.0):
        while(b_ > math.pi/2.0):
            b_ = b_ - math.pi
    return b_

def quaternion2yaw(q0,q1,q2,q3):
    return math.atan2(2*(q0*q3 + q1*q2),1-2*((q2)**2+(q3)**2))

def pose_callback(msg):
    global X, Y, th, V, W
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y

    q0 = msg.pose.pose.orientation.w
    q1 = msg.pose.pose.orientation.x
    q2 = msg.pose.pose.orientation.y
    q3 = msg.pose.pose.orientation.z

    th = quaternion2yaw(q0,q1,q2,q3)

    V = msg.twist.twist.linear.x
    W = msg.twist.twist.angular.z


    global strt, t_now
    if(strt==True):
        global strt, t_init
        strt = False
        t_init = time.time()
    t_now = time.time() - t_init
    # print(t_now)

r_scan = []
th_scan = []

x_scan = []
y_scan = []

def scan_callback(msg):
    global t_now
    # print(t_now)
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_increment = msg.angle_increment
    time_increment = msg.time_increment
    range_min = msg.range_min
    range_max = msg.range_max
    r = msg.ranges

    r_scan = []
    th_scan = []

    x_scan = []
    y_scan = []

    for i in range(0,len(r)):
        r_scan.append(r[i])
        th_scan.append(i*math.pi/180.0)

        global X, Y, th, V, W
        x_scan.append(X + r[i]*math.cos(th+i*math.pi/180.0))
        y_scan.append(Y + r[i]*math.sin(th+i*math.pi/180.0))

    print(type(r_scan), type(th_scan))


    # fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    # ax.plot(th_scan, r_scan)
    # ax.set_rmax(range_max*1.25)
    # # ax.set_rticks([0.5, 1, 1.5, 2])  # Less radial ticks
    # # ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
    # # ax.grid(True)

    # ax.set_title("Polar plot", va='bottom')
    # plt.show()

    # plt.plot(x_scan,y_scan,'og')
    # plt.show()

    plt.subplot(2,2,1)
    plt.plot(x_scan, y_scan,'b')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('cartesian plot')
    plt.grid(True)
    plt.axis('equal')

    plt.subplot(2,2,2)
    plt.plot(th_scan, r_scan,'b')
    plt.xlabel('th (rad)')
    plt.ylabel('radius (m)')
    plt.title('polar plot')
    plt.grid(True)


    plt.show()

def main():
    rospy.init_node('cluster_2d_lidar', anonymous=True)
    rospy.Subscriber('/odom', Odometry, pose_callback)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass