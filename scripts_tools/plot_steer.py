#!/usr/bin/env python


import math
import sys
import matplotlib.pyplot as plt
import numpy as np
# import rospy
import csv
import pandas as pd

# APOLLO_ROOT = os.path.join(os.path.dirname(__file__), '../../../')

# csv_file = csv.reader()
steer_angle = pd.read_csv(filepath_or_buffer='/home/cidi-xh/xuhao/Codes/github/cidi_gitlab/apollo_cidi/data/log/plot_steer_cmd.csv', sep=',')["steering_angle"].values
header_time = pd.read_csv(filepath_or_buffer='/home/cidi-xh/xuhao/Codes/github/cidi_gitlab/apollo_cidi/data/log/plot_steer_cmd.csv', sep = ',')["control_time"].values
steer_angle_cmd = pd.read_csv(filepath_or_buffer='/home/cidi-xh/xuhao/Codes/github/cidi_gitlab/apollo_cidi/data/log/plot_steer_cmd.csv', sep = ',')["steering_angle_cmd"].values
# print(lateral_error)
# print(header_time)
plt.figure()

plt.title('target steer/real steer(deg)')
plt.xlabel('header time(s)')
plt.ylabel('steer_angle')

plt.plot(header_time, steer_angle, 'r', header_time, steer_angle_cmd, 'g--')
# plt.subplot(3,1,3)
# plt.title('LQR Controller steering angle')
# plt.xlabel('header time(s)')
# plt.ylabel('steering angle(m)')
# plt.plot(header_time, steering_angle)

plt.legend(['steer_angle','steer_angle_cmd'], loc = 'upper right')
plt.show()

# plt.plot(x, y, 'ro', x, y1, 'g--')