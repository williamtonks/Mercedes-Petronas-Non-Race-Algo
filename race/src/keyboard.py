#!/usr/bin/env python

import rospy
from race.msg import drive_param # import the custom message
import curses
forward = 0
left = 0

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_talker', anonymous=True)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)

stdscr.refresh()

key = ''
while key != ord('q'):
        key = stdscr.getch()
        stdscr.refresh()

        # fill in the conditions to increment/decrement throttle/steer

        if key == curses.KEY_UP:
            if forward < 0:
                forward = 0
            elif forward == 0:
                forward = 5
            else:
                forward += 1
        elif key == curses.KEY_DOWN:
            if forward > 0:
                forward = 0
            elif forward == 0:
                forward = -5
            else:
                forward -= 1
        if key == curses.KEY_LEFT:
            if left > 0:
                left = 0
            else:
                left -= 5
        elif key == curses.KEY_RIGHT:
            if left < 0:
                left = 0
            else:
                left += 5
        elif key == curses.KEY_DC:
            # this key will center the steer and throttle
            forward = 0
            left = 0
        if forward >= 100:
            forward = 100
        if left >= 100:
            left = 100
        if forward <= -100:
            forward = -100
        if left <= -100:
            left = -100

        msg = drive_param()
        msg.velocity = forward
        msg.angle = left
        pub.publish(msg)

curses.endwin()
