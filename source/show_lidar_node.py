#!/usr/bin/env python3.8

import rospy
import lidar
from sensor_msgs.msg import LaserScan

if __name__ == '__main__':
    rospy.init_node('show_lidar')

    sub = rospy.Subscriber(
        '/scan', LaserScan, callback=lambda msg: lidar.get_lidar_array(msg, show=True)
        )
    
    rospy.spin()