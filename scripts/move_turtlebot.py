#!/usr/bin/env python

import math
import time

import rospy
from tf import transformations
from tf.transformations import euler_from_quaternion
from tf.transformations import math
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

scan: LaserScan = LaserScan()
on_goal_line: bool = False

ball_position = None
position_time = None
ball_velocity = None
counter = 10
done = False
xg = None

odom_once = False
lidar_once = False

init_position = None
orientation = None
position = None
roll = None
pitch = None
yaw = None
predicted_1 = None
predicted_2 = None
initial_mag = None
mag_prev = 100

isTurning = False
inPlace = True
position_test = [0, 0, 0]
position = [0, 0, 0]

def find_xy_position(distance, angle):
    return [distance*transformations.math.cos(angle), distance*transformations.math.sin(angle)]

def find_ball_velocity(position_1, position_2, time_1, time_2):
    change_in_time = time_2 - time_1
    velocity = []
    for i in range(len(position_1)):
        change_in_position = position_2[i] - position_1[i]
        velocity.append(change_in_position / change_in_time)
    return velocity

def predict_ball_position(pos_1, pos_2):
    m = (pos_2[1] - pos_1[1])/(pos_2[0] - pos_1[0])
    y_int = pos_1[1] - m * pos_1[0]

    x_G = -y_int/(m+(1/m))
    y_G = -(1/m)*x_G

    if x_G < 0:
        return [0, y_int, 0]
    
    return [x_G, y_G, 0]

def odom_callback(msg):
    global orientation, roll, pitch, yaw, position, init_position, odom_once
    odom_once = True
    orientation_msg = msg.pose.pose.orientation
    position_msg = msg.pose.pose.position
    orientation = [orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w]
    if (init_position == None):
        init_position = [position_msg.x, position_msg.y, position_msg.z]
    position = [position_msg.x - init_position[0], position_msg.y - init_position[1], position_msg.z - init_position[2]]
    (roll, pitch, yaw) = euler_from_quaternion(orientation)

def laserscan_callback(msg: LaserScan):
    global scan
    global lidar_once
    lidar_once = True
    scan = msg
    

def store_ball_position():
    global scan
    global ball_position
    global position_time
    minDistance = 500
    minAngle = 0
    ball_hits = []
    ball_angles = []
    for i in range(300, 360):
        if (scan.ranges[i] < scan.range_min or scan.ranges[i] > scan.range_max):
            continue
        if scan.ranges[i] < minDistance:
            minDistance = scan.ranges[i]
            minAngle = i
        if scan.ranges[i] < 10:
            ball_hits.append(scan.ranges[i])
            ball_angles.append(i)
    for i in range(0, 60):
        if (scan.ranges[i] < scan.range_min or scan.ranges[i] > scan.range_max):
            continue
        if scan.ranges[i] < minDistance:
            minDistance = scan.ranges[i]
            minAngle = i
        if scan.ranges[i] < scan.range_max:
            ball_hits.append(scan.ranges[i])
            ball_angles.append(i)
    if (len(ball_hits) < 1):
        return None
    ball_distance = ball_hits[transformations.math.floor(len(ball_hits)/2)]
    ball_angle = ball_angles[transformations.math.floor(len(ball_angles)/2)]*transformations.math.pi/180
    return find_xy_position(ball_distance, ball_angle)

def find_posts():
    global scan
    post_1_distance = 500
    post_2_distance = 500
    post_1_angle = 0
    post_2_angle = 0
    post_1_range = list()
    post_2_range = list()
    post_1_set = False
    post_2_set = False

    for i in range(len(scan.ranges)):
        if scan.ranges[i] > 5:
            if post_1_distance < 5:
                post_1_set = True
            if post_2_distance < 5:
                post_2_set = True
            continue
        if not post_1_set or post_2_set:
            post_1_range.append(i)
            if (scan.ranges[i] < post_1_distance):
                post_1_distance = scan.ranges[i]
                post_1_angle = i * math.pi / 180

        elif not post_2_set:
            post_2_range.append(i)
            if (scan.ranges[i] < post_2_distance):
                post_2_distance = scan.ranges[i]
                post_2_angle = i * math.pi / 180

    if not post_1_set:
        return None
    else:
        return {
            'post_1': {
                'angle': post_1_angle,
                'distance': post_1_distance
            },
            'post_2': {
                'angle': post_2_angle,
                'distance': post_2_distance
            }
        }

def go_to(destination, move_cmd):
    global position, yaw, isTurning, inPlace, initial_mag, mag_prev
    goal_direction = [x - y for x,y in zip(destination,position)]
    mag = math.sqrt(sum([x**2 for x in goal_direction]))
    if not initial_mag:
        initial_mag = mag

    goal_direction_angle = math.acos(float(goal_direction[0])/float(mag))
    if (goal_direction[1] < 0):
        goal_direction_angle =  - goal_direction_angle

    turn_angle = goal_direction_angle - yaw

    if turn_angle > math.pi:
        turn_angle = turn_angle - (2 * math.pi)
    if turn_angle < -math.pi:
        turn_angle = turn_angle + (2 * math.pi)
    
    if isTurning:
        if (abs(turn_angle) < 0.06):
            isTurning = False
            move_cmd.angular.z = 0
            
        else:

            
            if turn_angle < .5 and turn_angle > 0:
                move_cmd.angular.z = .3
            elif turn_angle > -.5 and turn_angle < 0:
                move_cmd.angular.z = -.3
            else:
                if turn_angle > 0:
                    move_cmd.angular.z = 1.5
                elif turn_angle < 0:
                    move_cmd.angular.z = -1.5
            
    else:
        move_cmd.linear.x = .22 #* mag / initial_mag
        move_cmd.angular.z = 0

    if mag < 0.15 or mag_prev+.01 < mag:
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        return True

    mag_prev = mag


##########  MAIN LOOP ##########
def goalie():
    global destination
    global position
    global isTurning
    global odom_once, lidar_once
    rospy.init_node('topic_publisher')
    move = Twist()

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    scan_sub = rospy.Subscriber('/scan', LaserScan, laserscan_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

    state = "PREDICT_BALL"
    rate = rospy.Rate(100)
    while (not (odom_once and lidar_once)):
        pass
    while not rospy.is_shutdown():
        if (state == "FIND_GOAL"):                              # FIND GOAL POSTS
            posts = find_posts()                             
            if posts:
                post_1_xy = find_xy_position(posts["post_1"]["distance"], posts["post_1"]["angle"])
                post_2_xy = find_xy_position(posts["post_2"]["distance"], posts["post_2"]["angle"])
                destination = [(post_1_xy[0]+post_2_xy[0])/2, (post_1_xy[1]+post_2_xy[1])/2, 0] 
                state = "GO_TO_GOAL"
                isTurning = True

            if not on_goal_line: # Find direction to goal line
                continue

        elif (state == "GO_TO_GOAL"):                           # MOVE TO DEFEND POSITION
            inPlace = go_to(destination, move)
            state = "PREDICT_BALL" if inPlace else state

        elif (state == "PREDICT_BALL"):                         # SCAN FOR AND PREDICT BALL MOVEMENT
            isTurning = True
            ranges_constr = list(scan.ranges[300:360]) + list(scan.ranges[0:60])
            did_loop_run = 0

            if min(ranges_constr) > 3:
                continue

            pos_1 = store_ball_position()
            if not pos_1:
                continue

            rospy.sleep(1)
            pos_2 = store_ball_position()
            if not pos_2:
                continue

            destination = predict_ball_position(pos_1, pos_2)
            mag_destination = math.sqrt(destination[0]**2 + destination[1]**2)
            if mag_destination < 0.0001:
                continue

            state = "BLOCK_BALL"                            
        elif (state == "BLOCK_BALL"):                           # MOVE TO BLOCK THE BALL
            go_to(destination, move)
            
        else:
            print("INVALID STATE")

        cmd_vel_pub.publish(move)
        rate.sleep()

if __name__ == "__main__":
    goalie()    