#!/usr/bin/env python

import rospy
from tf import transformations
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import time

scan: LaserScan = LaserScan()
on_goal_line: bool = False

ball_position = None
position_time = None
ball_velocity = None
counter = 10
done = False
xg = None

def find_ball_position(distance, angle):
    return [distance*transformations.math.cos(angle), distance*transformations.math.sin(angle)]

def find_ball_velocity(position_1, position_2, time_1, time_2):
    change_in_time = time_2 - time_1
    velocity = []
    for i in range(len(position_1)):
        change_in_position = position_2[i] - position_1[i]
        velocity.append(change_in_position / change_in_time)
    return velocity


def laserscan_callback(msg: LaserScan):
    global scan
    global xg
    global done
    global ball_position
    global position_time
    global ball_velocity
    global counter
    scan = msg
    # print(f'Min angle: {msg.angle_min}')
    # print(f'Max angle: {msg.angle_max}')
    # print(f'Angle Increment: {msg.angle_increment}')
    # print(f'# of ranges{len(msg.ranges)}')
    minDistance = 500
    minAngle = 0
    for i in range(len(msg.ranges)):
        if msg.ranges[i] < minDistance:
            minDistance = msg.ranges[i]
            minAngle = i
    # print(f'Closest object is {minDistance} away at {minAngle} degrees')
    if counter > 0:
        counter -= 1
        return
    counter = 10
    ball_hits = []
    ball_angles = []
    for i in range(len(msg.ranges)):
        if msg.ranges[i] < 10:
            ball_hits.append(msg.ranges[i])
            ball_angles.append(i)
    ball_distance = ball_hits[transformations.math.floor(len(ball_hits)/2)]
    ball_angle = ball_angles[transformations.math.floor(len(ball_angles)/2)]*transformations.math.pi/180
    new_position = find_ball_position(ball_distance, ball_angle)
    new_time = time.time()
    if ball_position:
        ball_velocity = find_ball_velocity(ball_position, new_position, position_time, new_time)
    ball_position = new_position
    position_time = new_time
    
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
                post_1_angle = i

        elif not post_2_set:
            post_2_range.append(i)
            if (scan.ranges[i] < post_2_distance):
                post_2_distance = scan.ranges[i]
                post_2_angle = i

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




def control_loop(xg):
    time_to_drive = xg / .2
    if (xg > 0):
        move.linear.x = .2
    elif (xg < 0):
        move.linear.x = -.2
    time.sleep(time_to_drive)
    move.linear.x = 0

rospy.init_node('topic_publisher')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
scan_sub = rospy.Subscriber('/scan', LaserScan, laserscan_callback)
rate = rospy.Rate(2)
move = Twist()




##########  MAIN LOOP ##########
while not rospy.is_shutdown():
    # Find posts
    posts = find_posts()
    if posts['post_1']['distance'] == 500:
        rate.sleep()
        continue

    if not on_goal_line: # Find direction to goal line
        print('get to goal line')
        continue
    
    # Rotate to be in line with goal

    # Find ball

    # Move to stop ball


    #pub.publish(move)
    rate.sleep()

    #control_loop(ball_direction)
    