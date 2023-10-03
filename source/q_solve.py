#!/usr/bin/env python3.8

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import lidar
import controll

import time
import random
import numpy as np
import itertools
from collections import namedtuple


def create_purpose(pos, distance):
    angles = np.arange(0, 360, 1)
    radians = np.radians(angles)
    x = pos[0] + distance * np.cos(radians)
    y = pos[1] + distance * np.sin(radians)
    
    # array of pairs like (x, y) for each point
    points = np.column_stack((x, y))

    mask = np.abs(points[:, 0]) < 2.2
    mask = np.abs(points[:, 1]) < 2.2

    return random.choice(points[mask])




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
        State = namedtuple('state', ['lidar', 'angle_to_purpose'])
        for lidar_state in itertools.product(range(len(self.danger_classes)+1), repeat=self.sectors):
            for purp_angle in range(len(self.angles_to_purpose)+1):
                s = State(lidar_state, purp_angle)
                self.q[s] = dict()
                for action in self.actions:
                    self.q[s][action] = 0
                    


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
        
        return r, done

    def choose_action(self):
        # epsilon greedy policy is here
        if random.random() > self.epsilon:
            return random.choice(list(self.q[self.current_state].keys()))
        return max(self.q[self.current_state], key=lambda a: self.q[self.current_state][a])

    def update(self, new_state, new_action, reward):
        self.previous_state = self.current_state
        self.current_state = new_state

        self.previous_action = self.current_action
        self.current_action = new_action

        # Q(s, a) = Q(s, a) + alpha * (r + gamma * max(Q(s', a')) - Q(s, a))
        self.q[self.previous_state][self.previous_action] += self.alpha * (reward + self.gamma * max(self.q[self.previous_state][a] for a in self.q[self.previous_state]) - self.q[self.previous_state][self.previous_action])


def main():

    q = Q_solver(
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