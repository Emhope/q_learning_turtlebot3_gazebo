#!/usr/bin/env python3.8

'''
functools for lidar processing
get_lidar_array func also used for reward calculating
'''

from sensor_msgs.msg import LaserScan
from matplotlib import pyplot as plt
import numpy as np
from numpy import pi


VIEWING_ANGLE = 120 # must de even
ANGLE = VIEWING_ANGLE // 2

# thats work for 360 degree lidar with 360 points
ANGLES = (np.arange(-ANGLE, ANGLE, 1) + 90) * pi / 180 


def get_lidar_array (msg: LaserScan, show=False) -> np.array:
    
    global ANGLE

    n = np.array(msg.ranges)
    n = np.concatenate((n[-ANGLE:], n[:ANGLE]))
    
    if not show:
        return n
    
    global ANGLES
    plt.clf()
    plt.xlim([-5, 5])
    plt.ylim([-5, 5])
    plt.plot(n * np.cos(ANGLES), n * np.sin(ANGLES), '.')
    plt.pause(0.01)
    plt.show(block=False)
    

def simplify_lidar(raw_data: np.array, sectors=3, distances=(0.4, 1)) -> tuple:
    size_of_sector = int(raw_data.shape[0]/sectors)
    
    sectors_mins = []
    for i in range(0, raw_data.shape[0], size_of_sector):
        
        # if that is not last sector
        if abs(i + size_of_sector - raw_data.shape[0]) > 3: 
            sectors_mins.append(min(raw_data[i:i+size_of_sector]))

        else:
            sectors_mins.append(min(raw_data[i:]))
            break
    
    return tuple(danger_class(i, distances) for i in sectors_mins)
    

def danger_class(distance: float, classes: list) -> int:
    for i, el in enumerate(classes):
        if distance < el :
            return i
    return len(classes) # len of classes is equal the low danger class
