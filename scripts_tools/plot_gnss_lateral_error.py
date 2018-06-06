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
lateral_error = pd.read_csv(filepath_or_buffer='/home/cidi-xh/xuhao/Codes/github/cidi_gitlab/apollo_cidi/data/log/lqr_steer_angle.csv', sep = ',')["lateral_error"].values
header_time = pd.read_csv(filepath_or_buffer='/home/cidi-xh/xuhao/Codes/github/cidi_gitlab/apollo_cidi/data/log/lqr_steer_angle.csv', sep = ',')["control_time"].values
heading_error = pd.read_csv(filepath_or_buffer='/home/cidi-xh/xuhao/Codes/github/cidi_gitlab/apollo_cidi/data/log/lqr_steer_angle.csv', sep = ',')["heading_error"].values
x = pd.read_csv(filepath_or_buffer='/home/cidi-xh/xuhao/Codes/github/cidi_gitlab/apollo_cidi/data/log/lqr_steer_angle.csv', sep = ',')["x"].values
y = pd.read_csv(filepath_or_buffer='/home/cidi-xh/xuhao/Codes/github/cidi_gitlab/apollo_cidi/data/log/lqr_steer_angle.csv', sep = ',')["y"].values
print(lateral_error)
print(header_time)
plt.figure()

plt.subplot(2,1,1)
plt.title('LQR Controller lateral errors')
plt.xlabel('header time(s)')
plt.ylabel('lateral error(m)/steer_angle')

plt.plot(header_time, lateral_error, 'r', header_time, steering_angle, 'g--')
plt.subplot(2, 1, 2)
plt.title('LQR Controller heading errors')
plt.xlabel('header time(s)')
plt.ylabel('heading error(m)')
plt.plot(header_time, heading_error)

# plt.subplot(3,1,3)
# plt.title('LQR Controller steering angle')
# plt.xlabel('header time(s)')
# plt.ylabel('steering angle(m)')
# plt.plot(header_time, steering_angle)

plt.legend()
plt.show()

# plt.plot(x, y, 'ro', x, y1, 'g--')