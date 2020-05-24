#!/usr/bin/env python
from __future__ import division
######################################################################
# IIIT Hyderabad 
######################################################################
#Pulkit Verma
######################################################################

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64, Int8
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped, PoseArray

import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import pi, cos, sin, atan, tan, atan2, sqrt, radians, degrees

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

#Variable Definition for Stanley
k = 0.1  # look forward gain
Lfc = 0.01  # look-ahead distance
Kp = 1.0  # speed propotional gain
dt = 0.05  # [s]
L = 0.1  # [m] wheel base of vehicle
target_speed = 0.1

show_animation = False

GUI_MIN_COORD = (0, 0)
GUI_MAX_COORD = (1000, 425)
ARENA_MIN_COORD = (0, 0)
ARENA_MAX_COORD = (10, 8)

# ROS Variables 
trajectory = []
tracking_status = None
# ****************** Stanley Functions ********************* 
# Update the Robot Location
def update(state, a, delta):
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt
    return state

def PIDControl(target, current):
    a = Kp * (target - current)
    return a


def pure_pursuit_control(state, cx, cy, pind):
    ind = calc_target_index(state, cx, cy)
    if pind >= ind:
        ind = pind
    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1
    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
    if state.v < 0:  # back
        alpha = math.pi - alpha
    Lf = k * state.v + Lfc
    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
    return delta, ind

def calc_target_index(state, cx, cy):
    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0
    Lf = k * state.v + Lfc

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cx[ind + 1] - cx[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
    return ind

# shutdown ROS on interrupt
def shutdown():
    global track_done
    track_done.publish(1)
    rospy.loginfo("Shutting Down Tracking")
    rospy.sleep(1)

def pub_odometry(data_x, data_y, theta, frame_id, child_frame_id):
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = frame_id
    odom_msg.child_frame_id = child_frame_id#child_frame_id
    odom_msg.pose.pose.position = Point(data_x, data_y, 0)
    odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,theta))
    print data_x, data_y
    return odom_msg

def cbTrajectory(msg):
    global trajectory
    for i in range(len(msg.poses)):
        trajectory.append([msg.poses[i].position.x, msg.poses[i].position.y])
        print len(trajectory)
    print "***************************************************"

def cbTrackStatus(msg):
    global tracking_status
    tracking_status = msg.data

def pixels2meter(pix, invert=True, GUI_MAX_COORD=GUI_MAX_COORD):
    x_vals = list(zip(*pix)[0])
    y_vals = list(zip(*pix)[1])

    M2m = 1000 #meter to mm
    XdistPerPix = ARENA_MAX_COORD[0] * M2m / GUI_MAX_COORD[0]
    YdistPerPix = ARENA_MAX_COORD[1] * M2m / GUI_MAX_COORD[1]
    cx = []
    cy = []
    for i in range(len(x_vals)):
        if  x_vals[i] > GUI_MAX_COORD[0] or \
            x_vals[i] < GUI_MIN_COORD[0] or \
            y_vals[i] > GUI_MAX_COORD[1] or \
            y_vals[i] < GUI_MIN_COORD[1]: 
            continue
        else:
            x_in_meters = x_vals[i] * XdistPerPix / M2m
            if invert:
                y_vals[i] = GUI_MAX_COORD[1] - y_vals[i]
            y_in_meters = y_vals[i] * YdistPerPix / M2m
            cx.append(round(x_in_meters, 3))
            cy.append(round(y_in_meters, 3))
    return {'x': cx, 'y': cy}   

def meter2pixel(met, invert=True, GUI_MAX_COORD=GUI_MAX_COORD):
    x_vals = list(zip(*met)[0])
    y_vals = list(zip(*met)[1])

    M2m = 1000 #meter to mm
    XPixPerMm = GUI_MAX_COORD[0] / (ARENA_MAX_COORD[0] * M2m)
    YPixPerMm = GUI_MAX_COORD[1] / (ARENA_MAX_COORD[1] * M2m)
    cx = []
    cy = []

    for i in range(len(x_vals)):
        if  x_vals[i] > ARENA_MAX_COORD[0] or \
            x_vals[i] < ARENA_MIN_COORD[0] or \
            y_vals[i] > ARENA_MAX_COORD[1] or \
            y_vals[i] < ARENA_MIN_COORD[1]: 
            continue
        else:
            x_in_pix = x_vals[i] * M2m * XPixPerMm
            if invert:
                y_vals[i] = ARENA_MAX_COORD[1] - y_vals[i]
            y_in_pix = y_vals[i] * M2m * YPixPerMm
            cx.append(x_in_pix)
            cy.append(y_in_pix)
    return {'x': cx, 'y': cy}


# **************** starting MAIN code *********************
# ROS initializations
rospy.init_node('tracking', anonymous=True)

odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
track_done = rospy.Publisher('/track_done', Int64, queue_size=10)
child_frame_id = rospy.get_param('~child_frame_id','base_link')
frame_id = rospy.get_param('~frame_id','world')

rospy.Subscriber('trajectory', PoseArray, cbTrajectory, queue_size=10)
rospy.Subscriber('start_tracking', Int8, cbTrackStatus)
rospy.on_shutdown(shutdown)
rate = rospy.Rate(10)
try:
    while(True):
        # rospy.wait_for_message("/trajectory", PoseArray)
        # rospy.wait_for_message("/start_tracking", Int8)
        if tracking_status == 1:
            pros_traj = pixels2meter(trajectory, invert=True, GUI_MAX_COORD=GUI_MAX_COORD)
            print pros_traj
            cx = pros_traj['x']
            cy = pros_traj['y']
            lastIndex = len(cx) - 1
            # initial state of the robot
            state = State(x=cx[0], y=cy[0], yaw=0, v=0.0)
            x = [state.x]
            y = [state.y]
            yaw = [state.yaw]
            v = [state.v]
            target_ind = calc_target_index(state, cx, cy)

            # MatplotLib initializations
            fig = plt.figure()
            ax2 = fig.add_subplot(111, aspect='equal')
            while (lastIndex > target_ind) and not rospy.is_shutdown() and sqrt( (cx[-1] - state.x)**2 + (cy[-1] - state.y)**2) > 0.1:
                print sqrt( (cx[-1] - state.x)**2 + (cy[-1] - state.y)**2)
                ai = PIDControl(target_speed, state.v)
                di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
                state = update(state, ai, di)
                x.append(state.x)
                y.append(state.y)
                yaw.append(state.yaw)
                v.append(state.v)
                pix_data = meter2pixel([[state.x, state.y]], invert=True, GUI_MAX_COORD = GUI_MAX_COORD)
                odom_data = pub_odometry(pix_data['x'][0], pix_data['y'][0], 0, frame_id, child_frame_id)
                odom_pub.publish(odom_data)
                if (tracking_status == 3):
                    trajectory = []
                    tracking_status = None
                    break

                # plot the values 
                if show_animation:
                    ax2.cla()
                    ax2.plot(cx, cy, ".r", label="course")
                    ax2.plot(x, y, "-b", label="trajectory")
                    ax2.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                    ax2.axis("equal")
                    plt.title("Speed[m/s]:" + str(state.v)[:4])
                    plt.pause(0.001)
            trajectory = []
            tracking_status = None

        elif (tracking_status == 2):
            trajectory = []
            tracking_status = None
            print "User does not want the robot to follow the trajectory"

except KeyboardInterrupt:
    print "Pressed Ctrl+C and the coded is completed"
    pass
    

