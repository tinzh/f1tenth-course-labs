#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

params = {}
pub = rospy.Publisher('error', pid_input, queue_size=10)

def callback(data):
	# angle of 0 is straight ahead, negative is right, positive is left
	def index_to_angle(index):
		return data.angle_min + data.angle_increment * index
		

	distances = list(data.ranges)

	# interpolate distances, NaNs get interpolated to closest adjacent distance
	first_nan_index = 0
	i = 0
	while i < len(distances):
		if math.isnan(distances[i]):
			first_nan_index = i
			while i < len(distances) and math.isnan(distances[i]):
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
		if abs(distances[i+1] - distances[i]) > params["disparity_threshold"]:
			if (distances[i] < distances[i+1]):
				disparities.append((i, i+1))
			else:
				disparities.append((i+1, i))

	disparities.sort(key=lambda disparity: distances[disparity[0]])

	for disparity in disparities:
		# if disparity was overwritten by other disparity
		if distances[disparity[1]] == distances[disparity[0]]:
			continue

		closest_distance = distances[disparity[0]]
		direction_sign = disparity[1] - disparity[0]

		index_width = 2 * math.arcsin(params["car_width"] / (4 * closest_distance))
		index_width = math.ceil(index_width)

		for i in range(disparity[0], 
				       disparity[0] + direction_sign * index_width, 
					   direction_sign):
			distances[i] = min(distances[i], closest_distance)
			

	# at this point, disparities have been extended in distances

	# directly find index with deepest gap
	deepest_gap = 0
	for i, distance in enumerate(distances):
		if not math.isnan(distance) and distance > distances[deepest_gap]:
			deepest_gap = i

			
	# TODO: maybe put upper bound on distance?
	# TODO: convert error to AckermannDrive
		

	desired_angle = index_to_angle(deepest_gap)


	msg = pid_input()
	msg.pid_error = desired_angle
	msg.pid_vel = params["velocity"]

	pub.publish(msg)


if __name__ == "__main__":
	def get_input(name, default_value):
		params[name] = float(raw_input("%s [%f]" % (name, default_value)) or str(default_value))

	get_input("disparity_threshold", 0.1)
	get_input("car_width", 0.5)
	get_input("velocity", 20)

	rospy.init_node('gap_finder',anonymous = True)
	rospy.Subscriber("/car_9/scan",LaserScan,callback)
	rospy.spin()
