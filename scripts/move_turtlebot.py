#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def laserscan_callback():
    print("boop")


def control_loop(direction):
    if ball_direction == "left":
        move.linear.x = .2

    if ball_direction == "right":
        move.linear.x = -.2

    if ball_direction == "center":
        move.angular.z = .2


rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
sub = rospy.Subscriber('/scan', LaserScan, laserscan_callback())
rate = rospy.Rate(2)
move = Twist()

ball_direction = "left"


##########  MAIN LOOP ##########
while not rospy.is_shutdown():
    pub.publish(move)
    rate.sleep()

    control_loop(ball_direction)
    