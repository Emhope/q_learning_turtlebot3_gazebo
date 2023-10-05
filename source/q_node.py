#!/usr/bin/env python3.8

import rospy
from geometry_msgs.msg import Twist
import json
from  nav_msgs.msg import Odometry
from std_msgs.msg import String
from math import (atan2, pi)

import q_solve
import controll

def get_purpose_angle(odom_msg: Odometry, purp_pose):
    
    pos = odom_msg.pose.pose.position

    angle = atan2(purp_pose[1] - pos.y, purp_pose[0] - pos.x) * 180 / pi
    if angle < 0:
        angle += 360

    return angle

    
def get_robot_angle(odom_msg: Odometry):

    quat = odom_msg.pose.pose.orientation

    # that work because robot can rotate only around z axis
    angle = atan2(quat.z, quat.w) * 180 / pi * 2
    if angle < 0:
        angle = 360 - abs(angle)
    
    return angle

def angle_from_robot_to_purp(odom_msg: Odometry, purp_pose):
    alphaR = get_robot_angle(odom_msg)
    alphaP = get_purpose_angle(odom_msg, purp_pose)

    # write code here



def main():
    
    actions = {
    0: controll.left,
    1: controll.forward,
    2: controll.right,
    }

    q = q_solve.Q_solver(
        alpha=0.4,
        gamma=0.999,
        epsilon=0.02,
        sectors=3,
        danger_classes=(0.4, 1.2),
        angles_to_purpose=(-15, 15),
        actions=actions
    )

    rospy.init_node('q_node')
    rospy.loginfo('q started')

    time = rospy.Time()
    rate = rospy.Rate(1)
    command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    distance_to_puspose = 1.2
    epoch = 0

    while not rospy.is_shutdown() and epoch < 1:
        controll.stop()
        epoch += 1
        rospy.loginfo(f'epoch {epoch} started')
        linear_speed = 0
        total_reward = 0
        done = False
        pos = (msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y)
        purpose_pos = q_solve.create_purpose(pos, distance_to_puspose)
        q.set_new_purpose(purpose_pos)

        first_it = True
        start = time.now()

        while not done and not rospy.is_shutdown():
            msg_lidar = rospy.wait_for_message('/turtle1/pose', String)
            msg_odom = rospy.wait_for_message('/prepared_lidar', Odometry)

            cmd = q.choose_action()
            linear_speed = actions[cmd]
            
            lidar_data = tuple(int(i) for i in msg_lidar.data.split())
            pos = (msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y)
            purppose_angle = angle_to_purp(...)

            r, done = q.get_reward(linear_speed, time.now() - start)
            total_reward += r
            start = time.now()
            state = q_solve.State(lidar=lidar_data, angle_to_purpose=purppose_angle)
            q.set_new_data(new_state=state, new_pos=pos)
            q.update(r)

            rate.sleep()
        rospy.loginfo(f'epoch ended with reward {total_reward}')

        
    


if __name__ == '__main__':
    main()