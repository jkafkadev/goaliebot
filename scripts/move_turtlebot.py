#!/usr/bin/env python

import rospy
from tf import transformations
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import time
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


def laserscan_callback(msg):
    global xg
    global done
    global ball_position
    global position_time
    global ball_velocity
    global counter
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
    print(ball_velocity)
    
    



def control_loop(xg):
    time_to_drive = xg / .2
    if (xg > 0):
        move.linear.x = .2
    elif (xg < 0):
        move.linear.x = -.2
    time.sleep(time_to_drive)
    move.linear.x = 0

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
sub = rospy.Subscriber('/scan', LaserScan, laserscan_callback)
rate = rospy.Rate(2)
move = Twist()

ball_direction = "left"


##########  MAIN LOOP ##########
while not rospy.is_shutdown():
    #pub.publish(move)
    rate.sleep()

    #control_loop(ball_direction)
    