#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#sub = rospy.Subscriber('/scan', String, callback)
rate = rospy.Rate(2)
move = Twist()
#move.linear.x = .1
#move.angular.z = .5

ball_direction = "right"

if ball_direction == "left":
    move.linear.x = .2

if ball_direction == "right":
    move.linear.x = -.2

if ball_direction == "center":
    move.linear.x = 0

while not rospy.is_shutdown():
    pub.publish(move)
    rate.sleep()

