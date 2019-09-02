#!/usr/bin/env python
import rospy
import os
from race.msg import drive_values
from race.msg import drive_param
from std_msgs.msg import Bool

"""
What you should do:
 1. Subscribe to the keyboard messages (If you use the default keyboard.py, you must subcribe to "drive_paramters" which is publishing messages of "drive_param")
 2. Map the incoming values to the needed PWM values
 3. Publish the calculated PWM values on topic "drive_pwm" using custom message drive_values
"""
forward = 0
left = 0
pub = rospy.Publisher("drive_pwm", drive_values, queue_size=10)
hostname = "192.168.1.1"


def callback(data):
    global forward, left, pub, hostname
    rospy.loginfo("Callback is called stupid")
    rospy.loginfo(data)
    forward = maprange((-100, 100), (6554, 13108), data.velocity)
    left = maprange((-100, 100), (6554, 13108), data.angle)
    # response = os.system("ping -c 1 " + hostname)
    # if response == 0:
    #     forward = 0
    #     left = 0
    msg = drive_values()
    msg.pwm_drive = forward
    msg.pwm_angle = left
    pub.publish(msg)

def maprange(a, b, s):
    (a1, a2), (b1, b2) = a, b
    return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

def listener():
    rospy.init_node('talker')
    rospy.Subscriber('drive_parameters',drive_param, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
