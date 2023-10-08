#!/usr/bin/env python3.8

'''
This node gets lidar data from '/scan' topic, simplifying it by splitting
to sectors and then publish simplified data to '/prepared_lidar' topic
'''

import rospy
import lidar

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

DANGER_CLASSES_LIDAR = (0.15, 0.4, 0.8)

def callback(msg: LaserScan):
    lidar_array = lidar.get_lidar_array(msg)
    simply_lidar = lidar.simplify_lidar(lidar_array, DANGER_CLASSES_LIDAR, 3)
    
    msg = String()
    msg.data = ' '.join(map(str, simply_lidar))
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('test_sub_node')
    rospy.loginfo('im started')

    rospy.Subscriber('/scan', LaserScan, callback=callback)
    pub = rospy.Publisher('/prepared_lidar', String, queue_size=10)

    rospy.spin()
