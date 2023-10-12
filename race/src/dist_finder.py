#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
desired_distance = 0.9	# distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15		# this vel variable is not really used here.
delay = 0.1     # delay in seconds
forward_projection = vel * delay / 30	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
forward_projection = 0.5
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
theta = math.radians(50)      # angle being swept (in degrees)

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
	angle -= math.pi/2
	index = (angle - data.angle_min) / data.angle_increment
	return data.ranges[int(index)]

def callback(data):
	a = getRange(data,theta) # obtain the ray distance for theta
	b = getRange(data,0)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)

	## Your code goes here to determine the projected error as per the alrorithm
	# Compute Alpha, AB, and CD..and finally the error.
	alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
	ab = b * math.cos(alpha)
	ac = forward_projection
	cd = ab + ac * math.sin(alpha)
	error = desired_distance - cd

	msg = pid_input()	# An empty msg is created of the type pid_input

	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)

	print("alpha: %lf, ab: %lf" % (alpha, ab))


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
        theta = math.radians(input("theta [50]: ") or 50)
	desired_distance = input("desired_distance [0.9]: ") or 0.9

	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_9/scan",LaserScan,callback)
	rospy.spin()
