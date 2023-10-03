#!/usr/bin/env python3.8

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import lidar
import controll

import time
import random
import itertools
from collections import namedtuple


actions = {
    0: controll.left,
    1: controll.forward,
    2: controll.right,
}

def distance(pos1, pos2):
    return ((pos1[0]- pos2[0]) ** 2 + (pos1[1]- pos2[1]) ** 2) ** 0.5


class Q_solver:
    def __init__(self, alpha, gamma, epsilon, sectors, danger_classes, angles_to_purpose, actions):

        

        self.sectors = sectors
        self.danger_classes = danger_classes
        self.angles_to_purpose = angles_to_purpose
        self.actions = actions
        self.alpha = alpha # learning rate/speed
        self.gamma = gamma # correction factor
        self.epsilon = epsilon # probabiliti of choose the best known action

        # QTable init
        self.q = dict()
        State = namedtuple('state', ['lidar', 'angle_to_purpose']) # idk where to paste it...
        for lidar_state in itertools.product(range(len(self.danger_classes)+1), repeat=self.sectors):
            for purp_angle in range(len(self.angles_to_purpose)+1):
                s = State(lidar_state, purp_angle)
                self.q[s] = dict()
                for action in self.actions:
                    self.q[s][action] = None
                    


        self.current_state = None
        self.previous_state = None
        
        self.current_action = None
        self.previous_action = None

    def get_reward(self, speed, timestamp, pos, previous_pos, purpose_pos, simplified_lidar):

        done = False
        r = 0

        # shortening the distance to purpose - good
        best_coming = speed * timestamp
        real_coming = distance(previous_pos, purpose_pos) - distance(pos, purpose_pos)
        r += 20 * (real_coming / best_coming)
        
        # close to obstacle - bad
        if 0 in simplified_lidar:
            r -= 50
        
        if distance(pos, purpose_pos) < 0.5:
            done = True
            r += 80

    def choose_action(self):
        ...

    def update(self):
        self.q[self.previous_state] += self.q.get(self.previous_state, 0)


def main():

    q = Q_solver(
        alpha=1,
        gamma=1,
        epsilon=1,
        sectors=3,
        danger_classes=(0.4, 1.2),
        angles_to_purpose=(-30, 30),
        actions=actions
    )
    print(q.q)

    rospy.init_node('q_node')
    rospy.loginfo('q started')
    
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    linear_speed = 0


    start = time.perf_counter()
    while not rospy.is_shutdown():
        if time.perf_counter() - start < 0.2:
            linear_speed = controll.right(pub, linear_speed)

        elif time.perf_counter() - start < 0.4:
            linear_speed = controll.left(pub, linear_speed)

        else:
            start = time.perf_counter()
    


if __name__ == '__main__':
    main()