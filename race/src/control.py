#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params

servo_offset = 18 # zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0

kp_vel = 0.0
kd_vel = 0.0
ki_vel = 0.0

min_vel = 15

num_seconds_to_look_at = 1
hz = 10
error_array_length = num_seconds_to_look_at * hz

error_array = [0]
error_sum = 0

last_speed = 0
last_steering = 0

# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 15 #TODO

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_9/offboard/command', AckermannDrive, queue_size = 1)

def bound(val, lower, upper):
	return min(upper, max(lower, val))

def control(data):
	global prev_error
	global vel_input
	kp = 120 #TODO
	kd = 0#1.5 #TODO
	ki = 0#12 #TODO
	global angle
	global error_sum
	global error_array
	global last_speed
	global last_steering

	print("PID Control Node is Listening to error")

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller

	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steering
	error = data.pid_error

	pid_steering_correction = kp * error + kd * (prev_error - error)*10 + ki*(error_sum)
	angle = pid_steering_correction

	vel_correction = kp_vel * abs(error) + kd_vel * abs(prev_error - error) 

	prev_error = error

	# if last_speed < 1:
	# error_array = [0]
	if len(error_array) > error_array_length:
		error_array = error_array[1:]

	if not math.isnan(error):
		error_array.append(error)

	error_sum = sum(error_array)*0.1#*len(error_array)

	print("error_sum: %lf" % (error_sum))

	# TODO: Make sure the steering value is within bounds [-100,100]
	steering_angle = angle + servo_offset
	# if math.isnan(steering_angle):
	# steering_angle = 
	steering_angle = bound(steering_angle, -100, 100)
	command.steering_angle = steering_angle

	# TODO: Make sure the velocity is within bounds [0,100]
	# max possible velocity is pid_vel, vel_correction decreases vel
	pid_vel = bound(data.pid_vel, 0, 100)
	vel = pid_vel - vel_correction
	vel = bound(vel, min_vel, pid_vel)
	command.speed = vel
	vel_coefficient = 25/(abs(steering_angle)+1)

	command.speed = min(max(min_vel, vel_coefficient * data.pid_vel), data.pid_vel)
	#vel_correction = 0
	if(abs(command.steering_angle) < 50):
		command.speed = data.pid_vel
	else:
		command.speed = min_vel

	print("angle: %lf", angle)

	print("steering_angle: %lf, vel_coefficient: %lf, speed: %lf, pid_vel: %lf, vel_correction: %lf" % (command.steering_angle, vel_coefficient, command.speed, data.pid_vel, vel_correction))
	last_speed = command.speed
	last_steering = command.steering_angle

	# Move the car autonomously
	command_pub.publish(command)

if __name__ == '__main__':

	# This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	kp = float(raw_input("Enter Kp Value [250]: ") or "250")
	kd = float(raw_input("Enter Kd Value [0]: ") or "0")
	ki = float(raw_input("Enter Ki Value [0]: ") or "0")

	kp_vel = float(raw_input("Enter vel Kp Value [0]: ") or "0")
	kd_vel = float(raw_input("Enter vel Kd Value [0]: ") or "0")
	ki_vel = float(raw_input("Enter vel Ki Value [0]: ") or "0")
	# vel_input = int(input("Enter desired velocity [15]: ") or "15")
	rospy.init_node('pid_controller', anonymous=True)
	# subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()

