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

def store_ball_position():
    global ball_position
    global predicted_1
    global predicted_2

    if predicted_1 == None:
        predicted_1 = ball_position
    elif predicted_2 == None:
        predicted_2 = ball_position


def predict_ball_position():
    global predicted_1
    global predicted_2

    m = (predicted_2[1] - predicted_1[1])/(predicted_2[0] - predicted_1[0])
    x_G = (-predicted_1[1]/m)+predicted_1[0]
    return x_G
    

    

def odom_callback(msg):
    global orientation, roll, pitch, yaw, position, init_position, odom_once
    odom_once = True
    orientation_msg = msg.pose.pose.orientation
    position_msg = msg.pose.pose.position
    orientation = [orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w]
    if (init_position == None):
        init_position = [position_msg.x, position_msg.y, position_msg.z]
    position = [position_msg.x - init_position[0], position_msg.y - init_position[1], position_msg.z - init_position[2]]
    #position = [0, 0, 0]
    (roll, pitch, yaw) = euler_from_quaternion(orientation)
    #print(position_msg.x)

def laserscan_callback(msg: LaserScan):
    global scan
    global xg
    global done
    global ball_position
    global position_time
    global ball_velocity
    global counter
    global lidar_once
    lidar_once = True
    scan = msg
    

def get_ball_info():
    # print(f'Min angle: {msg.angle_min}')
    # print(f'Max angle: {msg.angle_max}')
    # print(f'Angle Increment: {msg.angle_increment}')
    # print(f'# of ranges{len(msg.ranges)}')
    global scan
    global ball_position
    global position_time
    global ball_velocity
    global counter
    minDistance = 500
    minAngle = 0
    for i in range(len(scan.ranges)):
        if scan.ranges[i] < minDistance:
            minDistance = scan.ranges[i]
            minAngle = i
    # print(f'Closest object is {minDistance} away at {minAngle} degrees')
    if counter > 0:
        counter -= 1
        return
    counter = 10
    ball_hits = []
    ball_angles = []
    for i in range(len(scan.ranges)):
        if scan.ranges[i] < 10:
            ball_hits.append(scan.ranges[i])
            ball_angles.append(i)
    ball_distance = ball_hits[transformations.math.floor(len(ball_hits)/2)]
    ball_angle = ball_angles[transformations.math.floor(len(ball_angles)/2)]*transformations.math.pi/180
    new_position = find_xy_position(ball_distance, ball_angle)
    new_time = time.time()
    if ball_position:
        ball_velocity = find_ball_velocity(ball_position, new_position, position_time, new_time)
    ball_position = new_position
    position_time = new_time

def find_posts_v1():
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
    
def find_posts():
    global scan

    length = len(scan.ranges)
    stopI = 10000
    stopJ = -1

    post_1_ranges = []
    post_2_ranges = []

    for i in range(length):
        j = length - (i + 1)
        if (scan.ranges[i] < 5 and i < stopI):
            post_1_ranges.append(i)
            stopJ = -2
        elif (len(post_1_ranges) > 0 and stopJ == -2):
            stopJ = i

        if (scan.ranges[j] < 5 and j > stopJ):
            post_2_ranges.append(j)
            stopI = 10001
        elif (len(post_1_ranges) > 0 and stopI == 10001):
            stopI = j

    if (len(post_1_ranges) == 0 or len(post_2_ranges) == 0):
        return None

    post_1_index = post_1_ranges[int(len(post_1_ranges)/2)]
    post_2_index = post_2_ranges[int(len(post_2_ranges)/2)]

    post_1_distance = scan.ranges[post_1_index]
    post_1_angle = (post_1_index * scan.angle_increment) + scan.angle_min

    post_2_distance = scan.ranges[post_2_index]
    post_2_angle = (post_2_index * scan.angle_increment) + scan.angle_min

    print("p1_ang: " + str(post_1_angle))
    print("p2_ang: " + str(post_2_angle))
    print("test: " + str(post_1_ranges))
    print("testy: " + str(post_2_ranges))
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




def control_loop(xg, move):
    time_to_drive = xg / .2
    if (xg > 0):
        move.linear.x = .2
    elif (xg < 0):
        move.linear.x = -.2
    time.sleep(time_to_drive)
    move.linear.x = 0


def go_to(destination, move_cmd):
    global position, yaw, isTurning, inPlace
    goal_direction = [x - y for x,y in zip(destination,position)]
    mag = math.sqrt(sum([x**2 for x in goal_direction]))

    goal_direction_angle = math.acos(float(goal_direction[0])/float(mag))
    if (goal_direction[1] < 0):
        goal_direction_angle =  - goal_direction_angle

    turn_angle = goal_direction_angle - yaw

    if turn_angle > math.pi:
        turn_angle = turn_angle - (2 * math.pi)
    if turn_angle < -math.pi:
        turn_angle = turn_angle + (2 * math.pi)
    

    #move_cmd.angular.z = 0.1
    #print("goal: " + str(goal_direction_angle))
    #print("orient: " + str(yaw))
    print("turn: " + str(turn_angle))

    if isTurning:
        print("ye")
        if (abs(turn_angle) < 0.03):
            isTurning = False
            move_cmd.angular.z = 0
        else:
            move_cmd.angular.z = turn_angle
    else:
        print("ne")
        move_cmd.linear.x = .1
        move_cmd.angular.z = 0

    print("mag: " + str(mag))
    if mag < 0.2:
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        return True

"""     if (abs(turn_angle) < 0.1):
        move_cmd.angular.z = 0
    else:
        move_cmd.angular.z = turn_angle

    if (turn_angle < math.pi):
        move_cmd.linear.x = mag / 5
        pass """

""" 
    if mag < 0.2:
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        return True """


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

    state = "FIND_GOAL"
    rate = rospy.Rate(1)
    while (not (odom_once and lidar_once)):
        pass
    while not rospy.is_shutdown():
        if (state == "FIND_GOAL"):                              # FIND GOAL POSTS
            posts = find_posts_v1()                             # NOTE/TODO: Just need to make sure to note that the ball cant be too close during this process (ALSO REMOVE OTHER VERSION)
            if posts:
                # TODO: Convert posts to coordinates for bot to go to and store in destination
                post_1_xy = find_xy_position(posts["post_1"]["distance"], posts["post_1"]["angle"])
                post_2_xy = find_xy_position(posts["post_2"]["distance"], posts["post_2"]["angle"])
                print("post 1: " + str(post_1_xy))
                print("post 2: " + str(post_2_xy))
                destination = [(post_1_xy[0]+post_2_xy[0])/2, (post_1_xy[1]+post_2_xy[1])/2, 0] 
                print("position: " + str(position))
                state = "GO_TO_GOAL"
                #destination = [1, 0, 0]
                isTurning = True

            if not on_goal_line: # Find direction to goal line
                #print('get to goal line')
                continue

        elif (state == "GO_TO_GOAL"):                           # MOVE TO DEFEND POSITION
            inPlace = go_to(destination, move)
            print("destination" + str(destination))
            state = "PREDICT_BALL" if inPlace else state

        elif (state == "PREDICT_BALL"):                         # SCAN FOR AND PREDICT BALL MOVEMENT
            #pos_1 = store_ball_position()
            #rospy.sleep(2)
            #pos_2 = store_ball_position()
            #x_G = predict_ball_position()
            #r_G = [x_G, 0, 0]
            pass                                                # TODO: This. Store predicted location in destination
        elif (state == "BLOCK_BALL"):                           # MOVE TO BLOCK THE BALL
            go_to(destination, move)                            # NOTE/TODO: WILL NEED TO RESET POSITION TO BE RELATIVE TO ITSELF AGAIN (I THINK??)
        else:
            print("INVALID STATE")
        print(state)

        cmd_vel_pub.publish(move)
        rate.sleep()

        #control_loop(ball_direction, move)

if __name__ == "__main__":
    goalie()    