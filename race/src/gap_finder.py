#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

params = {}
pub = rospy.Publisher('error', pid_input, queue_size=10)
debug_pub = rospy.Publisher('debug_scan', LaserScan, queue_size=10)
heading_pub = rospy.Publisher('heading', LaserScan, queue_size=10)

def bound(val, lower, upper):
	return min(upper, max(lower, val))

def callback(data):
	# angle of 0 is straight ahead, negative is right, positive is left
	def index_to_angle(index):
		return data.angle_min + data.angle_increment * index
	
	def angle_to_index(angle):
		return int((angle - data.angle_min)/data.angle_increment)


	distances = list(data.ranges)

	# interpolate distances, NaNs get interpolated to closest adjacent distance
	first_nan_index = 0
	i = 0
	while i < len(distances):
		distances[i] = min(distances[i], params["max_distance"])
		if math.isnan(distances[i]) or distances[i] < 0.1:
			first_nan_index = i
			while i < len(distances) and (math.isnan(distances[i]) or distances[i] < 0.1):
				i += 1

			if first_nan_index == 0 or i == len(distances):
				min_distance = 0
			else:
				min_distance = min(distances[first_nan_index-1], distances[i])

			# if NaN gap width is big enough
			if i - first_nan_index > 5:
				min_distance = params["max_distance"]

			for j in range(first_nan_index, i):
				distances[j] = min_distance
		i += 1	

	# list of tuples (i, j) where there is a disparity between 
	# distances[i] and distances[j], with distances[i] < distances[j]
	disparities = []

	for i in range(len(distances)-1):
		if abs(index_to_angle(i)) > math.pi/2:
			continue
		if abs(distances[i+1] - distances[i]) > params["disparity_threshold"]:
			if (distances[i] < distances[i+1]):
				disparities.append((i, i+1))
			else:
				disparities.append((i+1, i))

	disparities.sort(key=lambda disparity: distances[disparity[0]])

	# extend disparities
	for disparity in disparities:
		closest_distance = distances[disparity[0]]

		# print(closest_distance)
		# print(params["car_width"] / (2 * closest_distance))

		# if domain error, don't extend disparities
		if params["car_width"] / 2 >= closest_distance:
			continue

		n = int(math.ceil(2 * math.asin(params["car_width"] / (4 * closest_distance)) / data.angle_increment))

		# print(n)

		disparity_sign = disparity[1] - disparity[0]
		for i in range(disparity[0], bound(disparity[0]+(n*disparity_sign), 0, len(distances)-1), disparity_sign):
			distances[i] = min(distances[disparity[0]], distances[i])

	# directly find index with deepest gap
	deepest_gap = angle_to_index(0)

	# num_indices_from_zero_to_ninety = angle_to_index(0)-angle_to_index(-math.pi/2)

	# print("dist straight ahead: ", distances[angle_to_index(0)])
	# print(len(distances))

	# window_left = angle_to_index(math.radians(-70))
	# window_right = angle_to_index(math.radians(30))

	# window_start = angle_to_index(math.radians(-60))

	# for i in range(window_start, window_left, -1):
	# 	if distances[i] > distances[deepest_gap]:
	# 		deepest_gap = i
	# for i in range(window_start, window_right):
	# 	if distances[i] > distances[deepest_gap]:
	# 		deepest_gap = i

	# if distances[deepest_gap] < params["max_distance"]:
	for i, distance in enumerate(distances):
		if distance > distances[deepest_gap] and -math.pi/2 < index_to_angle(i) < math.pi/2:
			deepest_gap = i

	i = deepest_gap
	j = deepest_gap

	# while i-1 > 0 and abs(distances[i-1] - distances[i]) < params["disparity_threshold"]: i -= 1
	# while j+1 < len(distances) and abs(distances[j+1] - distances[j]) < params["disparity_threshold"]: j += 1
	while i > 0 and distances[i] == params["max_distance"]:
		i -= 1
	while j+1 < len(distances) and distances[j] == params["max_distance"]:
		j += 1

	avg_deepest_gap = (i+j)/2
	deepest_gap = avg_deepest_gap
			
	# TODO: find widest max distance gap
	# TODO: interpolate nans better if their close
		
	print("DEEPEST GAP: ", deepest_gap)
	desired_angle = index_to_angle(deepest_gap)
	print("desired_angle: ", math.degrees(desired_angle))

	msg = pid_input()
	msg.pid_error = desired_angle

	if distances[deepest_gap] < params["max_distance"] or distances[angle_to_index(0)] < 0.75 :
		msg.pid_vel = params["velocity"] * 0.5
	else:
		msg.pid_vel = params["velocity"]
	msg.pid_vel = params["velocity"]

	pub.publish(msg)


    # public to debug topic for rviz
	# RVIZ, red dots representing the gap detection, uses laserscan message type but isn't actually a laserscan.

	#Not sure about these definitions
	index_width = 1
	gap_start_index = avg_deepest_gap - index_width
	gap_end_index = avg_deepest_gap + index_width
	
	#gap_start_index = max(0, gap_start_index)
	#gap_end_index = min(len(data.ranges) - 1, gap_end_index)
	
	#Satisfy laserscan
	coles_gap = LaserScan()
	coles_gap.header = data.header
	coles_gap.angle_min = index_to_angle(gap_start_index)
	coles_gap.angle_max = index_to_angle(gap_end_index)
	coles_gap.ranges = [data.ranges[i] for i in range(gap_start_index, gap_end_index)] #+ 1)]
	coles_gap.angle_increment = data.angle_increment
	coles_gap.time_increment = data.time_increment
	coles_gap.scan_time = data.scan_time
	coles_gap.range_min = data.range_min
	coles_gap.range_max = data.range_max

	heading_pub.publish(coles_gap)

	justins_gap = data
	data.ranges = distances
	debug_pub.publish(justins_gap)


if __name__ == "__main__":
	def get_input(name, default_value):
		params[name] = float(raw_input("%s [%f]: " % (name, default_value)) or str(default_value))

	get_input("disparity_threshold", 0.1)
	get_input("car_width", 0.5)
	get_input("velocity", 30)
	get_input("max_distance", 2.5)

	rospy.init_node('gap_finder',anonymous = True)
	rospy.Subscriber("/car_9/scan",LaserScan,callback)
	rospy.spin()

