#!/usr/bin/env python

import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

params = {}

# PID state
prev_error = 0.0
error_array = [0]
error_sum = 0

command_pub = rospy.Publisher('/car_9/offboard/command', AckermannDrive, queue_size = 1)

def bound(val, lower, upper):
	return min(upper, max(lower, val))

def control(data):
	global prev_error
	global error_array
	global error_sum

	kp = params["kp"]
	kd = params["kd"]
	ki = params["ki"]

	error = data.pid_error
	pid_steering_correction = (kp * error 
							   + kd * (prev_error - error) * params["hz"] 
							   + ki * (error_sum))

	# store kd state
	prev_error = error

	# store ki state
	if not math.isnan(error):
		error_array.append(error)
		if len(error_array) > params["error_array_length"]:
			error_array = error_array[1:]
		error_sum = sum(error_array) / params["hz"]


	angle = bound(pid_steering_correction + params["servo_offset"], -100, 100)

	# tuples of (threshold, proportion of max_vel)
	speed = data.pid_vel
	thresholds = [(55, 1/3), (80, 3/4)]
	for threshold, proportion in thresholds:
		if abs(angle) < threshold:
			speed *= proportion
			break

	command = AckermannDrive()
	command.steering_angle = angle
	command.speed = speed

	print("steering_angle: ", angle, "speed: ", speed)
	command_pub.publish(command)


if __name__ == '__main__':
	def get_input(name, default_value):
		params[name] = float(raw_input("%s [%f]" % (name, default_value)) or str(default_value))

	get_input("kp", 100)
	get_input("kd", 0.05)
	get_input("ki", 0.4)

	params["servo_offset"] = 18
	params["num_seconds_to_look_at"] = 0.5
	params["hz"] = 10
	params["error_array_length"] = params["num_seconds_to_look_at"] * params["hz"]

	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
