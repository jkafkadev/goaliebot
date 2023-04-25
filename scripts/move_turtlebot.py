#!/usr/bin/env python

import rospy
from tf import transformations
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
ball_position = [0, 0, 0]

def find_ball_position(distance, angle):
    return [distance*transformations.math.cos(angle), distance*transformations.math.sin(angle)]


def laserscan_callback(msg):
    ball_hits = []
    ball_angles = []
    for i in range(len(msg.ranges)):
        if msg.ranges[i] < 10:
            ball_hits.append(msg.ranges[i])
            ball_angles.append(i)
    ball_distance = ball_hits[transformations.math.floor(len(ball_hits)/2)]
    ball_angle = ball_angles[transformations.math.floor(len(ball_angles)/2)]*transformations.math.pi/180
    print(find_ball_position(ball_distance, ball_angle))
    
    



def control_loop(direction):
    if ball_direction == "left":
        move.linear.x = .2

    if ball_direction == "right":
        move.linear.x = -.2

    if ball_direction == "center":
        move.angular.z = .2


rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
sub = rospy.Subscriber('/scan', LaserScan, laserscan_callback)
rate = rospy.Rate(2)
#move = Twist()

ball_direction = "left"


##########  MAIN LOOP ##########
while not rospy.is_shutdown():
    #pub.publish(move)
    rate.sleep()

    #control_loop(ball_direction)
    