#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
import tf

# Global variables for storing the path, path resolution, frame ID, and car details
params              = {}
plan                = []
path_resolution     = []
frame_id            = 'map'
car_name            = str(sys.argv[1])
trajectory_name     = str(sys.argv[2])

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size = 1)
polygon_pub         = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size = 1)

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon

wp_seq          = 0
control_polygon = PolygonStamped()

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


# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN       = 0.325

def purepursuit_control_node(data):
    # Main control function for pure pursuit algorithm

    # Create an empty ackermann drive message that we will populate later with the desired steering angle and speed.
    command = AckermannDrive()

    global wp_seq
    global curr_polygon

    # Obtain the current position of the race car from the inferred_pose message
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y


    # TODO 1: The reference path is stored in the 'plan' array.
    # Your task is to find the base projection of the car on this reference path.
    # The base projection is defined as the closest point on the reference path to the car's current position.
    # Calculate the index and position of this base projection on the reference path.
    
    # Your code here

    def calc_distance(x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        return math.sqrt(dx*dx + dy*dy)

    min_index = -1
    min_distance = 10000
    for i, (x, y) in enumerate(plan):
        distance = calc_distance(x, y, odom_x, odom_y)
        if (distance < min_distance):
            min_index = i
            min_distance = distance

    pose_x, pose_y = plan[min_index]

    
    # Calculate heading angle of the car (in radians)
    heading = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                        data.pose.orientation.y,
                                                        data.pose.orientation.z,
                                                        data.pose.orientation.w))[2]
    

    # TODO 2: You need to tune the value of the lookahead_distance
    lookahead_distance = params["lookahead_distance"]


    # TODO 3: Utilizing the base projection found in TODO 1, your next task is to identify the goal or target point for the car.
    # This target point should be determined based on the path and the base projection you have already calculated.
    # The target point is a specific point on the reference path that the car should aim towards - lookahead distance ahead of the base projection on the reference path.
    # Calculate the position of this goal/target point along the path.

    # Your code here

    i = min_index
    while calc_distance(plan[i][0], plan[i][1], odom_x, odom_y) < lookahead_distance:
        i = (i + 1) % len(plan)

    far_index = i
    close_index = (i - 1 + len(plan)) % len(plan)

    x1, y1 = plan[close_index]
    x2, y2 = plan[far_index]
    xc, yc = odom_x, odom_y

    m = (y2-y1) / (x2-x1)
    j = y1 - m*x1
    k = j + yc
    
    a = (m*m + 1)
    b = (2*k*m - 2*xc)
    c = (k*k + xc*xc - lookahead_distance*lookahead_distance)

    x_poss1 = (-b - math.sqrt(b*b - 4*a*c)) / (2*a)
    x_poss2 = (-b + math.sqrt(b*b - 4*a*c)) / (2*a)

    # use solution inside x1
    target_x = x_poss1 if x1 < x_poss1 < x2 or x2 < x_poss1 < x1 else x_poss2
    target_y = m*target_x + j


    # TODO 4: Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.
    # Your code here

    # q0, q1, q2, q3 = data.pose.orientation
    # curr_angle = math.atan2(2 * (q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3)
    curr_angle = heading
    desired_angle = math.atan2(target_y - odom_y, target_x - odom_x)
    alpha = desired_angle - curr_angle
    steering_angle = math.atan(2 * WHEELBASE_LEN * math.sin(alpha) / lookahead_distance)
    turning_radius = lookahead_distance / (2*math.sin(alpha))

    print("alpha: {}\tsteering angle: {}\tturning radius: {}".format(math.degrees(alpha), math.degrees(steering_angle), turning_radius))


    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    # Your code here    
    # TODO: fix alignment
    # TODO: translate from actual steering angle to msg value (-100 to 100) (maybe with turning radius?)
    command.steering_angle = 0.0


    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed
    speed = params["speed"]
    thresholds = [(80, 1.0/2), (30, 3.0/4)]
    for threshold, proportion in thresholds:
        if abs(command.steering_angle) > threshold:
            speed *= proportion
            break
    command.speed = speed


    command_pub.publish(command)

    # Visualization code
    # Make sure the following variables are properly defined in your TODOs above:
    # - odom_x, odom_y: Current position of the car
    # - pose_x, pose_y: Position of the base projection on the reference path
    # - target_x, target_y: Position of the goal/target point
    base_link    = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    base_link.x    = odom_x
    base_link.y    = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points  = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq      = wp_seq
    control_polygon.header.stamp    = rospy.Time.now()
    wp_seq = wp_seq + 1
    polygon_pub.publish(control_polygon)


if __name__ == '__main__':
    try:
        def get_input(name, default_value):
            params[name] = float(raw_input("%s [%f]" % (name, default_value)) or str(default_value))

        get_input("lookahead_distance", 1.0)
        get_input("speed", 25)

        rospy.init_node('pure_pursuit', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.spin()
    except rospy.ROSInterruptException:

        pass
