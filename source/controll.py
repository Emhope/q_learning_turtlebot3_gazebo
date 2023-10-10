#!/usr/bin/env python3.8

import rospy
from geometry_msgs.msg import Twist


FORWARD_SPEED = 0.22
BACKWARD_SPEED = -0.6
ROT_SPEED = 0.3

def make_cmd(pub: rospy.Publisher, lin, ang):
    cmd = Twist()
    cmd.linear.x = lin
    cmd.angular.z = ang

    pub.publish(cmd)

def stop(pub: rospy.Publisher):
    make_cmd(pub, 0.0, 0.0)

def forward(pub: rospy.Publisher, speed):
    
    current_speed = min(FORWARD_SPEED, max(speed + 0.02, 0))
    make_cmd(pub, current_speed, 0.0)
    return current_speed

def backward(pub: rospy.Publisher, speed):
    
    # current_speed = max(BACKWARD_SPEED, min(speed - 0.06, 0))
    current_speed = -0.06
    make_cmd(pub, current_speed, 0.0)
    return current_speed

def left(pub: rospy.Publisher, speed):
    
    current_speed = max(min(FORWARD_SPEED, speed + 0.01), 0.01)
    make_cmd(pub, current_speed * 0.5, ROT_SPEED)
    return current_speed

def right(pub: rospy.Publisher, speed):
    
    current_speed = max(min(FORWARD_SPEED, speed + 0.01), 0.01)
    make_cmd(pub, current_speed * 0.5, -ROT_SPEED)
    return current_speed
