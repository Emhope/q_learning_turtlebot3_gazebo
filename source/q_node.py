#!/usr/bin/env python3.8

import rospy
import json
from geometry_msgs.msg import Twist
from  nav_msgs.msg import Odometry
from std_msgs.msg import String
from math import (atan2, pi)

import q_solve
import controll
import angle_tools
import lidar_processing_node


def main():
    cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    actions = {
    0: controll.left,
    1: controll.forward,
    2: controll.backward,
    3: controll.right,
    }

    q = q_solve.Q_solver(
        alpha=0.4,
        gamma=0.999,
        epsilon=0.5,
        sectors=3,
        danger_classes=lidar_processing_node.DANGER_CLASSES_LIDAR,
        angles_to_purpose=(-15, 15),
        actions=actions
    )

    rospy.init_node('q_node')
    rospy.loginfo('q started')

    time = rospy.Time()
    rate = rospy.Rate(1)
    distance_to_puspose = 1.2
    epoch = 0

    while not rospy.is_shutdown() and epoch < 100:
        controll.stop(cmd_publisher)
        epoch += 1
        
        linear_speed = 0
        total_reward = 0
        done = False

        msg_lidar = rospy.wait_for_message('/prepared_lidar', String)
        msg_odom = rospy.wait_for_message('/odom', Odometry)
        
        pos = (msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y)
        lidar_data = tuple(int(i) for i in msg_lidar.data.split())
        purpose_pos = q_solve.create_purpose(pos, distance_to_puspose)
        purpose_angle = angle_tools.angle_from_robot_to_purp(msg_odom, purpose_pos)

        q.set_new_purpose(purpose_pos)
        q.set_new_data(lidar_data=lidar_data, angle_to_purp=purpose_angle, new_pos=pos)

        rospy.loginfo(f'''
                      epoch {epoch} started
                      purpose: {purpose_pos}''')

        start = time.now()

        while not done and not rospy.is_shutdown():
            msg_lidar = rospy.wait_for_message('/prepared_lidar', String)
            msg_odom = rospy.wait_for_message('/odom', Odometry)

            cmd = q.choose_action()
            linear_speed = actions[cmd](cmd_publisher, linear_speed)

            #rospy.loginfo(f'im choose action {actions[cmd]}, speed = {linear_speed}')
            
            lidar_data = tuple(int(i) for i in msg_lidar.data.split())
            pos = (msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y)
            purpose_angle = angle_tools.angle_from_robot_to_purp(msg_odom, purpose_pos)

            r, done = q.get_reward(linear_speed, time.now() - start)
            total_reward += r
            start = time.now()
    
            q.set_new_data(lidar_data=lidar_data, angle_to_purp=purpose_angle, new_pos=pos)
            q.update(r)

            rate.sleep()
            
        q.save(f'q_table{epoch}.pkl')
        rospy.loginfo(f'epoch ended with reward {total_reward}')
        
    
if __name__ == '__main__':
    main()
    