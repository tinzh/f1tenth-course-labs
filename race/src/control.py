#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params
kp = 50 #TODO
kd = 0.0 #TODO
ki = 0.0 #TODO
servo_offset = 18	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0

kp_vel = 0.0
kd_vel = 0.0
ki_vel = 0.0

min_vel = 10


# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 15	#TODO

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_9/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle

	print("PID Control Node is Listening to error")

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller

	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steering
	error = data.pid_error

	pid_steering_correction = kp * error + kd * (prev_error - error)
	angle = pid_steering_correction

	vel_correction = kp_vel * abs(error) + kd_vel * (abs(prev_error) - abs(error))

	prev_error = error

        

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	steering_angle = angle + servo_offset
	steering_angle = min(100, max(-100, steering_angle))
	command.steering_angle = steering_angle

	# TODO: Make sure the velocity is within bounds [0,100]
	pid_vel = min(100, max(0, data.pid_vel))
	vel = pid_vel - vel_correction
	vel = min(pid_vel, max(min_vel, vel))
	command.speed = vel

	print("steering_angle: %lf, speed: %lf, pid_vel: %lf, vel_correction: %lf" % (command.steering_angle, command.speed, pid_vel, vel_correction))

	# Move the car autonomously
	command_pub.publish(command)

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	kp = float(input("Enter Kp Value [250]: ") or "250")
	kd = float(input("Enter Kd Value [0]: ") or "0")
	ki = float(input("Enter Ki Value [0]: ") or "0")

	kp_vel = float(input("Enter vel Kp Value [0]: ") or "0")
	kd_vel = float(input("Enter vel Kd Value [0]: ") or "0")
	ki_vel = float(input("Enter vel Ki Value [0]: ") or "0")
	# vel_input = int(input("Enter desired velocity [15]: ") or "15")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
