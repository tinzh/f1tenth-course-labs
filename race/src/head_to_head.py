#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped, Polygon, Point32, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import tf

# Global variables
params              = {}
plan                = []
path_resolution     = []
frame_id            = 'map'
car_name            = 'car_9'
trajectory_name     = str(sys.argv[1])
wp_seq              = 0                 # id for stamp
lidar_data          = LaserScan()

command_pub         = rospy.Publisher('/car_9/offboard/command', AckermannDrive, queue_size = 1)
polygon_pub         = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size = 1)

STEERING_RANGE = 100.0
WHEELBASE_LEN       = 0.325

def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    file_path = os.path.expanduser('/home/nvidia/depend_ws/src/F1tenth_car_workspace/f1tenth-course-labs/{}.csv'.format(trajectory_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    # Convert string coordinates to floats and calculate path resolution
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    for index in range(1, len(plan)):
         dx = plan[index][0] - plan[index-1][0]
         dy = plan[index][1] - plan[index-1][1]
         path_resolution.append(math.sqrt(dx*dx + dy*dy))

# callback for lidar topic
def record_lidar(data):
    global lidar_data
    lidar_data = data

def index_to_angle(index):
    return lidar_data.angle_min + lidar_data.angle_increment * index

def angle_to_index(angle):
    return int((angle - lidar_data.angle_min)/lidar_data.angle_increment)

def get_ftg_target(target_heading):
    distances = lidar_data.ranges
    
    # convert all small distances to NaN
    for i, d in enumerate(distances):
        if d < 0.1:
            distances[i] = float('nan')

    # interpolate distances with NaNs, set upper distance bound to lookahead_distance
    i = 0
    while i < len(distances):
        distances[i] = min(distances[i], params["lookahead_distance"])
        
        # get gap of NaNs
        if math.isnan(distances[i]):
            # [first_nan_index, i) will be range of NaNs
            first_nan_index = i
            while i < len(distances) and math.isnan(distances[i]):
                i += 1

            if i - first_nan_index > 5:
                min_distance = params["lookahead_distance"]
            elif first_nan_index == 0 or i == len(distances):
                min_distance = 0
            else:
                min_distance = min(
                    distances[first_nan_index-1],
                    distances[i], 
                    params["lookahead_distance"]
                )

            # fill in NaNs with interpolated value
            for j in range(first_nan_index, i):
                distances[j] = min_distance

            i -= 1
        elif distances[i] > params["lookahead_distance"]:
            distances[i] = params["lookahead_distance"]

        i += 1

    # find disparities
    disparities = []
    for i in range(angle_to_index(-math.pi/2), angle_to_index(math.pi/2)):
        delta = distances[i+1] - distances[i]
        if delta > params["disparity_threshold"]:
            disparities.append((i, i+1))
        elif -delta > params["disparity_threshold"]:
            diisparities.append((i+1, i))

    # extend disparities (closest to farthest)
    disparities.sort(key=lambda disparity: distances[disparity[0]])
    for disparity in disparities:
        d = distances[disparity[0]]
        n = int(math.ceil(2 * math.asin(params["car_width"]/(4*d)) / data.angle_increment))
        if disparity[1] - disparity[0] == 1: 
            # extend to the right
            for i in range(disparity[0], min(len(distances), disparity[0] + n)):
                distances[i] = min(d, distances[i])
        else: 
            # extend to the left
            for i in range(disparity[0], max(0, disparity[0] - n)):
                distances[i] = min(d, distances[i])


    # TODO: find target

def get_purepursuit_target(odom_x, odom_y):
    def calc_square_distance(x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        return dx*dx + dy*dy

	# Find reference point on plan
    min_index = -1
    min_square_distance = 10000
    for i, (x, y, _, _) in enumerate(plan):
        square_distance = calc_square_distance(x, y, odom_x, odom_y)
        if (square_distance < min_square_distance):
            min_index = i
            min_square_distance = square_distance
    pose_x, pose_y, _, _ = plan[min_index]

	# Get target point ahead on path
    i = min_index % len(path_resolution)
    curr_distance = 0
    while curr_distance < params["lookahead_distance"]:
        curr_distance += path_resolution[i]
        i = (i+1) % len(path_resolution)
    target_x, target_y, _, _ = plan[i]

    return target_x, target_y

# Main decision-making callback
def control(data):
	# Current position of car
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y

    # Use pure pursuit
    target_x, target_y = calc_purepursuit_target(odom_x, odom_y)

    # Calculate current heading angle (radians)
    q = data.pose.orientation
    heading = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

	# Get alpha from position and target
    desired_angle = math.atan2(target_y - odom_y, target_x - odom_x)
    alpha = desired_angle - heading
    steering_angle = math.atan(2 * WHEELBASE_LEN * math.sin(alpha) / params["lookahead_distance"])
    turning_radius = WHEELBASE_LEN / math.tan(alpha)

    print("alpha: {}\tsteering angle: {}".format(math.degrees(alpha), math.degrees(steering_angle)))

    ##########################################################################

    # Construct AckermannDrive message
    command = AckermannDrive()

	# Publish steering angle
    left_max = math.radians(31.5)
    right_max = math.radians(24.5)
    if steering_angle < 0: 
        command.steering_angle = max(-100.0, steering_angle * 100.0 / left_max)
    else: 
        command.steering_angle = min(100.0, steering_angle * 100.0 / right_max)

	# Publish speed (with velocity scaling)
    speed = params["speed"]
    thresholds = [(80, 1.0/2), (30, 3.0/4)]
    for threshold, proportion in thresholds:
       if abs(command.steering_angle) > threshold:
           speed *= proportion
           break
    command.speed = speed

    command_pub.publish(command)

    ##########################################################################

    # Visualization code
    # - odom_x, odom_y: Current position of the car
    # - pose_x, pose_y: Position of the base projection on the reference path
    # - target_x, target_y: Position of the goal/target point
    global wp_seq
    base_link    = Point32(odom_x, odom_y, 0)
    nearest_pose = Point32(pose_x, pose_y, 0)
    nearest_goal = Point32(target_x, target_y, 0)
	control_polygon = PolygonStamped(Header(wp_seq, rospy.Time.now(), frame_id), Polygon([nearest_pose, base_link, nearest_goal]))
	wp_seq = wp_seq + 1
	polygon_pub.publish(control_polygon)


if __name__ == '__main__':
    def get_input(name, default_value):
        params[name] = float(raw_input("%s [%f]" % (name, default_value)) or str(default_value))

    get_input("speed", 45)
    get_input("lookahead_distance", 1.5)
    get_input("disparity_threshold", 0.1)
    get_input("car_width", 0.5)


    rospy.init_node('pure_pursuit', anonymous = True)

    rospy.loginfo('obtaining trajectory')
    construct_path()

    rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, control)
    rospy.Subscriber("/car_9/scan", LaserScan, record_lidar)

    rospy.spin()
