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
import json
import pickle


State = namedtuple('state', ['lidar', 'angle_to_purpose'])


def create_purpose(pos, distance):
    angles = np.arange(0, 360, 1)
    radians = np.radians(angles)
    x = pos[0] + distance * np.cos(radians)
    y = pos[1] + distance * np.sin(radians)
    
    # array of pairs like (x, y) for each point
    points = np.column_stack((x, y))

    mask = np.abs(points[:, 0]) < 2.2
    mask &= np.abs(points[:, 0]) > -2.2
    mask &= np.abs(points[:, 1]) < 2.2
    mask &= np.abs(points[:, 1]) > -2.2

    return random.choice(points[mask])


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
        
        for lidar_state in itertools.product(range(len(self.danger_classes)+1), repeat=self.sectors):
            for purp_angle in range(len(self.angles_to_purpose)+1):
                s = (lidar_state, purp_angle)
                self.q[s] = dict()
                for action in self.actions:
                    self.q[s][action] = 0

        self.current_state = None
        self.previous_state = None
        
        self.current_action = None
        self.previous_action = None

        self.pos = None
        self.previous_pos = None
        self.purpose_pos = None

    def reset_table(self):
        self.q = dict()
        
        for lidar_state in itertools.product(range(len(self.danger_classes)+1), repeat=self.sectors):
            for purp_angle in range(len(self.angles_to_purpose)+1):
                s = (lidar_state, purp_angle)
                self.q[s] = dict()
                for action in self.actions:
                    self.q[s][action] = 0

    def set_new_purpose(self, purpose_pos):
        self.purpose_pos = purpose_pos

        self.current_state = None
        self.previous_state = None
        
        self.current_action = None
        self.previous_action = None

        self.current_pos = None
        self.previous_pos = None


    def set_new_data(self, lidar_data, angle_to_purp, new_pos):
        simplified_angle = lidar.danger_class(angle_to_purp, self.angles_to_purpose)
        new_state = (lidar_data, simplified_angle) 

        self.previous_state = self.current_state
        self.current_state = new_state
        self.previous_pos = self.current_pos
        self.current_pos = new_pos

    def get_reward(self, speed: float, timestamp: rospy.Duration, collision: bool):

        if self.previous_pos is None:
            return 0, False

        done = False
        r = 0

        if collision:
            r -= 0.05
        # best_coming = speed * timestamp.to_sec()
        real_coming = distance(self.previous_pos, self.purpose_pos) - distance(self.current_pos, self.purpose_pos)
        # print(f'real_coming: {real_coming}')
        r += 1.0 * real_coming
        # shortening the distance to purpose - good
        
        if speed < 0:
            r -= 0.05
        
        if self.current_state[0][1] == 0 and speed > 0:
            r -= 1

        # close to obstacle - bad
        if 0 in self.current_state[0]: # 0 in lidar is very close
            r -= 0.05
        
        if distance(self.current_pos, self.purpose_pos) < 0.25:
            done = True
            r += 0.08
        
        return r, done

    def choose_action(self):
        # epsilon greedy policy is here

        sorted_actions = sorted(list(self.q[self.current_state].items()), key=lambda i: i[1]) # sort actions in state by reward


        if random.random() > self.epsilon or sorted_actions[0] == sorted_actions[-1]: # if epsion or unknown state
            action = random.choice(sorted_actions[:-1])[0]
        else:
            action = sorted_actions[-1][0]

        self.previous_action = self.current_action
        self.current_action = action

        return self.current_action
         

    def update(self, reward):
        # if that is the first update
        if self.previous_action is None:
            return

        # Q(s, a) = Q(s, a) + alpha * (r + gamma * max(Q(s', a')) - Q(s, a))
        self.q[self.previous_state][self.previous_action] += self.alpha * (reward + self.gamma * max(self.q[self.previous_state][a] for a in self.q[self.previous_state]) - self.q[self.previous_state][self.previous_action])


    def save(self, filename='q_table.pkl'):
        with open(filename, 'wb') as file:
            pickle.dump(self.q, file)
    
    def upload(self, filename):
        with open(filename, 'rb') as file:
            self.q = pickle.load(file)


