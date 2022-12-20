#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

global v_linear_min
v_linear_min = 0.1

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

global wp_x, wp_y, N_wp
wp_x = []
wp_y = []
N_wp = 0

global N_wp_covered
N_wp_covered = 0

global des_vel
des_vel = Twist()

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

def get_traj_input(x, y):
    global wp_x, wp_y
    global N_wp
    N_wp = len(wp_x)

    global N_wp_covered

    di = math.sqrt((wp_x[N_wp_covered] - x)**2 + (wp_y[N_wp_covered] - y)**2)
    reach_zone = 0.1
    if(di<=reach_zone):
        global N_wp_covered
        N_wp_covered = N_wp_covered + 1

    if(N_wp_covered>=N_wp):
        global N_wp_covered
        N_wp_covered = N_wp-1


    return wp_x[N_wp_covered], wp_y[N_wp_covered]

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

def a2b(X_, Y_, th_, x0_, y0_):
    k1 = 0.15
    k2 = 2/(math.pi)*k1
    d  = math.sqrt((x0_- X_)**2 + (y0_- Y_)**2)
    V_ = k1 * d
    alpha = math.atan2((y0_- Y_), (x0_- X_))
    b = alpha - th_
    beta = wrap_pi(b)

    W_= k2 * beta

    return V_, W_

def a2b2(X_, Y_, th_, x0_, y0_):
    k1 = 0.15
    k2 = 0.75*1.25
    d  = math.sqrt((x0_- X_)**2 + (y0_- Y_)**2)
    V_ = k1 * d
    alpha = math.atan2((y0_- Y_), (x0_- X_))
    b = alpha - th_
    beta = wrap_pi(b)
    dirn_frwrd = (beta> -math.pi/2.0) and (beta<= math.pi/2.0) 
    beta = wrap_pi_2(b)
    W_= k2 * beta

    global N_wp, N_wp_covered
    if(N_wp_covered<N_wp-1):
        if(V_<v_linear_min):
            V_ = v_linear_min

    if(dirn_frwrd):
        return V_, W_
    else:
        return -V_, W_

    return V_, W_

def a2b3(X_, Y_, th_, x0_, y0_, th_0_):
    k1 = 0.15
    k2 = 0.75
    k3 = 1.0

    d  = math.sqrt((x0_- X_)**2 + (y0_- Y_)**2)
    V_ = k1 * d
    alpha = math.atan2((y0_- Y_), (x0_- X_))
    b = alpha - th_
    beta = wrap_pi(b)
    dirn_frwrd = (beta> -math.pi/2.0) and (beta<= math.pi/2.0) 
    beta = wrap_pi_2(b)
    gamma = wrap_pi(th_0_ - th_)

    W_= k2 * beta + k3 *limit((1.0/d), -1.0, 1.0 )* gamma

    # print(beta*180.0/(math.pi), gamma*180.0/(math.pi))

    # V_ = v_linear

    if(dirn_frwrd):
        return V_, W_
    else:
        return -V_, W_
    

def quaternion2yaw(q0,q1,q2,q3):
    return math.atan2(2*(q0*q3 + q1*q2),1-2*((q2)**2+(q3)**2))

def pose_callback(msg):
    global X, Y
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y

    q0 = msg.pose.pose.orientation.w
    q1 = msg.pose.pose.orientation.x
    q2 = msg.pose.pose.orientation.y
    q3 = msg.pose.pose.orientation.z

    V = msg.twist.twist.linear.x
    W = msg.twist.twist.angular.z


    global th
    th = quaternion2yaw(q0,q1,q2,q3)

    # print( X, Y, th*180/(math.pi))
    
    global des_vel
    des_vel.linear.x = 0.0
    des_vel.angular.z = 0.0

    global strt
    if(strt==True):
        global strt, t_init
        strt = False
        t_init = time.time()
    t_now = time.time() - t_init

    # Goal position
    x0 = 1.28
    y0 = -1.8
    th_0 = 45.0 * math.pi/180.0

    x0, y0 = get_traj_input(X, Y)
    print(x0, y0)
    
    # des_V, des_W = a2b3(X, Y, th, x0, y0, th_0)
    des_V, des_W = a2b2(X, Y, th, x0, y0)
    
    v_limit = 0.26
    w_limit = 1.82
    des_V = limit(des_V, -v_limit, v_limit)
    des_W = limit(des_W, -w_limit, w_limit)

    #print(des_V, des_W)
    # print( X, Y, th*180/(math.pi), des_V, des_W)

    des_vel.linear.x  = des_V
    des_vel.angular.z = des_W

    pub.publish(des_vel)

def read_waypoints(file_name):
    file_wp = open("wp.txt","r") # read mode
    wp_data = file_wp.readlines()
    file_wp.close()

    n_wp = len(wp_data)

    for i in range(0,n_wp):
        wp_i = wp_data[i]
        wp_i = wp_i.split()
        wp_ix = wp_i[0]
        wp_ix = wp_ix[:len(wp_ix)-1]
        wp_iy = wp_i[1]
        wp_ix = float(wp_ix)
        wp_iy = float(wp_iy)
        # print(wp_ix, wp_iy)
        global wp_x, wp_y
        wp_x.append(wp_ix)
        wp_y.append(wp_iy)
    # print(wp_x)
    # print(wp_y)


def main():
    rospy.init_node('waypoint_follower', anonymous=True)
    read_waypoints("wp.txt")
    rospy.Subscriber('/odom', Odometry, pose_callback)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass