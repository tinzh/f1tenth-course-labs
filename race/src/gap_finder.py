#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

params = {}
pub = rospy.Publisher('error', pid_input, queue_size=10)
debug_pub = rospy.Publisher('debug_scan', LaserScan, queue_size=10)

def callback(data):
	# angle of 0 is straight ahead, negative is right, positive is left
	def index_to_angle(index):
		return data.angle_min + data.angle_increment * index
		

	distances = list(data.ranges)

	# interpolate distances, NaNs get interpolated to closest adjacent distance
	first_nan_index = 0
	i = 0
	while i < len(distances):
		if math.isnan(distances[i]) or distances[i] < 0.1:
			first_nan_index = i
			while i < len(distances) and (math.isnan(distances[i]) or distances[i] < 0.1):
				i += 1

			if first_nan_index == 0 or i == len(distances):
				min_distance = 0
			else:
				min_distance = min(distances[first_nan_index-1], distances[i])

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

	# for disparity in disparities:
	# 	# if disparity was overwritten by other disparity
	# 	if distances[disparity[1]] == distances[disparity[0]]:
	# 		continue

	# 	closest_distance = distances[disparity[0]]
	# 	direction_sign = disparity[1] - disparity[0]
        
	# 	# print(index_to_angle(disparity[0]), index_to_angle(disparity[1]))
	# 	# print(closest_distance)
	# 	# print(params["car_width"] / 4 / closest_distance)
	# 	index_width = 2 * math.asin(params["car_width"] / (4.0 * closest_distance))
	# 	index_width = int(math.ceil(index_width))

	# 	for i in range(disparity[0], 
	# 			       disparity[0] + direction_sign * index_width, 
	# 				   direction_sign):
	# 		distances[i] = min(distances[i], closest_distance)

	for disparity in disparities:
		n = math.ceil(params["car_width"]/(2*distances[disparity[0]]*math.tan(data.angle_increment)))
		for i in range(disparity[0], disparity[0]+(n*(disparity[1]-disparity[0])), disparity[1]-disparity[0]):
			distances[i] = min(distances[disparity[0]], distances[i])

	# at this point, disparities have been extended in distances

	# directly find index with deepest gap
	deepest_gap = 0
	for i, distance in enumerate(distances):
		if not math.isnan(distance) and distance > distances[deepest_gap]:
			deepest_gap = i

			
	# TODO: maybe put upper bound on distance?
	# TODO: convert error to AckermannDrive
		

	desired_angle = index_to_angle(deepest_gap)
	print(desired_angle)

	msg = pid_input()
	msg.pid_error = desired_angle
	msg.pid_vel = params["velocity"]

	pub.publish(msg)


    # public to debug topic for rviz
	# RVIZ, red dots representing the gap detection, uses laserscan message type but isn't actually a laserscan.

	#Not sure about these definitions
	index_width = 10
	gap_start_index = deepest_gap - index_width
	gap_end_index = deepest_gap + index_width
	
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

	debug_pub.publish(coles_gap)



if __name__ == "__main__":
	def get_input(name, default_value):
		params[name] = float(raw_input("%s [%f]" % (name, default_value)) or str(default_value))

	get_input("disparity_threshold", 0.1)
	get_input("car_width", 0.5)
	get_input("velocity", 20)

	rospy.init_node('gap_finder',anonymous = True)
	rospy.Subscriber("/car_9/scan",LaserScan,callback)
	rospy.spin()

