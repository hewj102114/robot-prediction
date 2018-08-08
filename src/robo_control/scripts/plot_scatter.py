#!/usr/bin/python2.7
# coding=utf-8
# !/usr/bin/python2.7
# !/home/ubuntu/anaconda2/bin/python2.7
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt
import cPickle as pickle

with open('ukf.txt') as f:
    ukf_dict = pickle.load(f)

with open('team_info.txt') as f:
    team_info_dict = pickle.load(f)

fig = plt.figure()

ukf_time, ukf_x_vel, ukf_y_vel = [], [], []
for time, values in ukf_dict.items():
    ukf_time.append(time)
    ukf_x_vel.append(values[0])
    ukf_y_vel.append(values[1])

# ukf_x = plt.scatter(ukf_time, ukf_x_vel, c='b', marker='o')
ukf_y = plt.scatter(ukf_time, ukf_y_vel, c='b', marker='+')

team_info_time, team_info_x_vel, team_info_y_vel = [], [], []
for time, values in team_info_dict.items():
    team_info_time.append(time)
    team_info_x_vel.append(values[0])
    team_info_y_vel.append(values[1])

# team_info_x = plt.scatter(team_info_time, team_info_x_vel, c='r', marker='o')
team_info_y = plt.scatter(team_info_time, team_info_y_vel, c='r', marker='+')

plt.axis([-0.5, 25, -1.5, 1.5])
# plt.legend((ukf_x, ukf_y, team_info_x, team_info_y), ("ukf_x", "ukf_y", "team_info_x", "team_info_y"))
plt.show()