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


def save_callback(_, q: q_solve.Q_solver):
    #q.epsilon *= 1.01
    rospy.loginfo(f'autosave, epsilon increace to {q.epsilon}')
    q.save(f'last_save.pkl')


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
        gamma=0.9,
        epsilon=0.5,
        sectors=3,
        danger_classes=lidar_processing_node.DANGER_CLASSES_LIDAR,
        angles_to_purpose=(-15, 15),
        actions=actions
    )
    q.upload('/home/misha/practice_ws/src/q_learning/q_learning_turtlebot3_gazebo/q_table8.pkl')
    
    rospy.init_node('q_node')
    rospy.loginfo('q started')

    time = rospy.Time()
    rate = rospy.Rate(1)
    distance_to_puspose = 1.2
    epoch = 0

    timer = rospy.Timer(rospy.Duration(secs=180), lambda i:save_callback(i, q))

    while not rospy.is_shutdown() and epoch < 20:
        controll.stop(cmd_publisher)
        epoch += 1
        
        linear_speed = 0
        total_reward = 0
        done = False

        msg_lidar = rospy.wait_for_message('/prepared_lidar', String)
        msg_odom = rospy.wait_for_message('/odom', Odometry)
        
        pos = (msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y)
        lidar_data = tuple(int(i) for i in msg_lidar.data.split()[:-1])
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
            
            collision = float(msg_lidar.data.split()[-1]) < 0.15
            lidar_data = tuple(int(i) for i in msg_lidar.data.split()[:-1])
            pos = (msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y)
            purpose_angle = angle_tools.angle_from_robot_to_purp(msg_odom, purpose_pos)

            r, done = q.get_reward(linear_speed, time.now() - start, collision)
            total_reward += r
            print(total_reward, ' ' * 10, end='/r')
            start = time.now()
    
            q.set_new_data(lidar_data=lidar_data, angle_to_purp=purpose_angle, new_pos=pos)
            q.update(r)

            rate.sleep()
            
        q.save(f'q_table{epoch}.pkl')
        q.epsilon += 0.05 
        rospy.loginfo(f'epoch ended with reward {total_reward}')
        
        
    
if __name__ == '__main__':
    main()
    