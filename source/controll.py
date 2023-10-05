#!/usr/bin/env python3.8

import rospy
from geometry_msgs.msg import Twist


FORWARD_SPEED = 0.22
ROT_SPEED = 0.3

def make_cmd(pub: rospy.Publisher, lin, ang):
    cmd = Twist()
    cmd.linear.x = lin
    cmd.angular.z = ang

    pub.publish(cmd)

def stop(pub: rospy.Publisher, speed):
    make_cmd(pub, 0.0, 0.0)

def forward(pub: rospy.Publisher, speed):
    global FORWARD_SPEED
    current_speed = min(FORWARD_SPEED, speed + 0.02)
    make_cmd(pub, current_speed, 0.0)
    return current_speed

def left(pub: rospy.Publisher, speed):
    global FORWARD_SPEED
    global ROT_SPEED
    current_speed = min(FORWARD_SPEED, speed + 0.01)
    make_cmd(pub, current_speed * 0.5, -ROT_SPEED)
    return current_speed

def right(pub: rospy.Publisher, speed):
    global FORWARD_SPEED
    global ROT_SPEED
    current_speed = min(FORWARD_SPEED, speed + 0.01)
    make_cmd(pub, current_speed * 0.5, ROT_SPEED)
    return current_speed
