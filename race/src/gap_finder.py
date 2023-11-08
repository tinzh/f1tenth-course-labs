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
	has_deep_gap = False
	while i < len(distances):
		if distances[i] > params["max_distance"]:
			distances[i] = params["max_distance"]
			has_deep_gap = True
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

		# if domain error for asin, don't extend disparities
		if params["car_width"] / 2 >= closest_distance:
			continue

		n = int(math.ceil(2 * math.asin(params["car_width"] / (4 * closest_distance)) / data.angle_increment))

		disparity_sign = disparity[1] - disparity[0]
		for i in range(disparity[0], bound(disparity[0]+(n*disparity_sign), 0, len(distances)-1), disparity_sign):
			distances[i] = min(distances[disparity[0]], distances[i])

	# find widest gap with max_distance, otherwise just find deepest gap
	if has_deep_gap:
		widest_gap = (0, 0) # (i, j) where there is a gap in range [i, j)
		i = 0
		while i < len(distances):
			if -math.pi/2 < index_to_angle(i) < math.pi/2:
				i+=1
				continue
			if distances[i] == params["max_distance"]:
				gap_start = i
				while i < len(distances) and distances[i] == params["max_distance"]:
					i += 1
				gap_end = i
				if gap_end - gap_start > widest_gap[1] - widest_gap[0]:
					widest_gap = (gap_start, gap_end)
			i += 1

		deepest_gap = (widest_gap[0] + widest_gap[1]) / 2
	else:
		deepest_gap = angle_to_index(0)
		for i, distance in enumerate(distances):
			if distance > distances[deepest_gap] and -math.pi/2 < index_to_angle(i) < math.pi/2:
				deepest_gap = i
			
	print("DEEPEST GAP: %d" % (deepest_gap))
	desired_angle = index_to_angle(deepest_gap)
	print("desired_angle: %f" % (math.degrees(desired_angle)))

	msg = pid_input()
	msg.pid_error = desired_angle

	if distances[deepest_gap] < params["max_distance"] or distances[angle_to_index(0)] < 0.75 :
		msg.pid_vel = params["velocity"] * 0.5
	else:
		msg.pid_vel = params["velocity"]
	msg.pid_vel = params["velocity"]

	pub.publish(msg)

    # publish to debug topic for rviz
	# RVIZ, red dots representing the gap detection, uses laserscan message type but isn't actually a laserscan.

	# Not sure about these definitions
	index_width = 1
	gap_start_index = deepest_gap - index_width
	gap_end_index = deepest_gap + index_width
	
	# Satisfy laserscan
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
	get_input("velocity", 40)
	get_input("max_distance", 2.5)

	rospy.init_node('gap_finder',anonymous = True)
	rospy.Subscriber("/car_9/scan",LaserScan,callback)
	rospy.spin()
