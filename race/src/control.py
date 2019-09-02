#!/usr/bin/env python

import rospy
from race.msg import drive_param
from race.msg import pid_input

kp = 14.0
kd = 0.09
servo_offset = 18.5	# zero correction offset in case servo is misaligned. 
prev_error = 0.0 
vel_input = 10.0	# arbitrarily initialized. 25 is not a special value. This code can input desired velocity from the user.

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

"""
def maprange(a, b, s):
    (a1, a2), (b1, b2) = a, b
    if s > a2:
	s = a2
    elif s < a1:
	s = a1
    return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))
"""


def control(data):
	global prev_error
	global vel_input
	global kp
	global kd

	## Your code goes here
	# 1. Scale the error 
	# 2. Apply the PID equation on error to compute steering
	# 3. Make sure the steering value is within bounds for talker.py
	error = data.pid_error
	distance_error = kp * error + kd * (prev_error - error)
	#angle = int(maprange((-10,10),(-100,100), distance_error))
	angle = -10 * distance_error
	#if (angle < 0):
		#angle = angle * 3
	if angle < -100:
		angle = -100
	elif angle > 100:
		angle = 100
	prev_error = error

	rospy.loginfo(error)
	rospy.loginfo(distance_error)
	rospy.loginfo(angle)
	rospy.loginfo("-------------")
	

	## END
	msg = drive_param()
	msg.velocity = vel_input	
	msg.angle = angle
	pub.publish(msg)

def shutdown_car():
	vel_input = 0
	angle = 0
	msg = drive_param()
	msg.velocity = vel_input
	msg.angle = angle
	pub.publish(msg)


if __name__ == '__main__':
	global kp
	global kd
	global vel_input
	print("Listening to error for PID")
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	vel_input = input("Enter Velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.on_shutdown(shutdown_car)
	rospy.spin()
