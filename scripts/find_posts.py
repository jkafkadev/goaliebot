from sensor_msgs.msg import LaserScan
from tf.transformations import math
def find_posts(scan: LaserScan):
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