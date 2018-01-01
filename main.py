#!/usr/bin/env python

import rospy
import os
import cv2
import numpy as np
import binascii
import struct
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import time

dist = -1
pc2_msg = -1
min_dist = -1

def update_dist(msg):
	global dist,min_dist
	dist = msg.ranges[len(msg.ranges)/2]
	min_dist = min(msg.ranges)


def update_dist_from_red(msg):
	global pc2_msg
	pc2_msg = msg


def move_forward(twist_publisher):
	move_msg = Twist()
	
	orig_dist = dist

	if min_dist >= 0.5:
		while abs(dist - orig_dist) < 0.40:
			move_msg.linear.x = (0.5 - abs(dist - orig_dist))/1.1
			twist_publisher.publish(move_msg)
	else:
		print "Can't move"

	time.sleep(5)
	#print abs(dist - orig_dist)


def turn_arround(twist_publisher):
	print "Enter wanted angel: "
	degrees = float(raw_input())
	turn(twist_publisher,degrees)


def turn(twist_publisher,degrees):
	rads = math.radians(degrees)

	turn_msg = Twist()
	turn_msg.angular.z = -0.3

	current_time = rospy.Time.now().to_sec()
	while (rospy.Time.now().to_sec() - current_time < rads * 3):
		twist_publisher.publish(turn_msg)


def get_distance_print(twist_publisher):
	dist_from_red = get_distance()
	print dist_from_red

def get_distance():
	global pc2_msg

	dist_from_red = None
	distances = []

	for i in xrange(0, pc2_msg.width * pc2_msg.height):
	    point = map(lambda z: int(binascii.hexlify(z),16), struct.unpack("ccc", pc2_msg.data[i*32+16:i*32+19]))
	    hsv_point = cv2.cvtColor(np.uint8([[point]]), cv2.COLOR_BGR2HSV)
	    red_range = cv2.inRange(hsv_point, (0,100,100), (10,255,255))
	    if(red_range == 255):
	    	z = struct.unpack("<f", pc2_msg.data[i*32+8:i*32+12])
	    	if(z[0] >= 0): 
	    		distances.append(z[0])

	if(len(distances)>0):
		dist_from_red = min(distances)

	return dist_from_red


def find_red_object(twist_publisher):
	
	while True:
		dist_from_red = get_distance()
		if not dist_from_red:
			turn(twist_publisher, 45)
		else:
			print "Found"
			break

	print dist_from_red


def print_menu():
	print "\n\n"
	print "Choose an option:"
	print "1) Move Forward"
	print "2) Turn Around"
	print "3) Get Distance To Red Object"
	print "4) Find Red Object\n"


def get_option():
	print "Enter your choice:"
	return raw_input()


def handle_option(opt, twist_publisher):
	opts = {
		'1': move_forward,
		'2': turn_arround,
		'3': get_distance_print,
		'4': find_red_object
	}

	if opt in opts.keys():
		opts[opt](twist_publisher)



if __name__ == '__main__':
    try:
    	rospy.init_node('ass2', anonymous=True)
    	twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    	rospy.Subscriber("scan", LaserScan, update_dist)
    	rospy.Subscriber("torso_camera/depth_registered/points", PointCloud2, update_dist_from_red)

    	opt = None
    	while True:
			print_menu()
			opt = get_option()
			handle_option(opt, twist_publisher)

    except Exception as e:
    	print e
        pass




