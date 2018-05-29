#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
Record localization,planning.error data
"""

import atexit
import logging
import math
import os
import sys

import rospy
from gflags import FLAGS

from logger import Logger
from modules.canbus.proto import chassis_pb2
from modules.localization.proto import localization_pb2
from modules.planning.proto import planning_pb2
from modules.control.proto import control_cmd_pb2


class RecordPlot(object):
    """
    record and plot class
    """

    def write(self, data):
        """wrap file write function to flush data to disk"""
        self.file_handler.write(data)
        self.file_handler.flush()

    def __init__(self, record_file):
        self.firstvalid = False
        self.logger = Logger.get_logger("RecordPlot")
        self.record_file = record_file
        self.logger.info("Record file to: " + record_file)

        try:
            self.file_handler = open(record_file, 'w')
        except:
            self.logger.error("open file %s failed" % (record_file))
            self.file_handler.close()
            sys.exit()

        # self.write("x,y,z,speed,acceleration,curvature,"\
        #                 "curvature_change_rate,time,theta,gear,s,throttle,brake,steering\n")
        # self.write("x,y,heading,real_speed,control_time,steering,lateral_error,heading_error\n")#plan_x,plan_y
        self.write("control_time,steering,steer_feedback,steer_forward,lateral_error,heading_error\n")  #

        self.localization = localization_pb2.LocalizationEstimate()
        self.chassis = chassis_pb2.Chassis()
        self.chassis_received = False
        self.planning = planning_pb2.ADCTrajectory()
        self.control_cmd = control_cmd_pb2.ControlCommand()

        self.cars = 0.0
        self.startmoving = False

        self.terminating = False
        self.carcurvature = 0.0

        self.prev_carspeed = 0.0
        self.control_time = 0.0
        self.lateral_error = 0.0
        self.heading_error = 0.0

    def chassis_callback(self, data):
        """
        New message received
        """
        if self.terminating == True:
            self.logger.info("terminating when receive chassis msg")
            return

        self.chassis.CopyFrom(data)
        #self.chassis = data
        if math.isnan(self.chassis.speed_mps):
            self.logger.warning("find nan speed_mps: %s" % str(self.chassis))
        if math.isnan(self.chassis.steering_percentage):
            self.logger.warning(
                "find nan steering_percentage: %s" % str(self.chassis))
        self.chassis_received = True

    def control_callback(self,data):
        """
        New message received
        """
        if self.terminating == True:
            self.logger.info("terminating when receive control msg")
            return
        self.control_cmd.CopyFrom(data)
        self.control_time = self.control_cmd.header.timestamp_sec
        self.lateral_error = self.control_cmd.debug.simple_lat_debug.lateral_error
        self.heading_error = self.control_cmd.debug.simple_lat_debug.heading_error


    def localization_callback(self, data):
        """
        New message received
        """
        if self.terminating == True:
            self.logger.info("terminating when receive localization msg")
            return

        if not self.chassis_received:
            self.logger.info(
                "chassis not received when localization is received")
            return

        self.localization.CopyFrom(data)
        #self.localization = data
        carx = self.localization.pose.position.x
        cary = self.localization.pose.position.y
        carz = self.localization.pose.position.z
        cartheta = self.localization.pose.heading
        if math.isnan(self.chassis.speed_mps):
            self.logger.warning("find nan speed_mps: %s" % str(self.chassis))
            return
        if math.isnan(self.chassis.steering_percentage):
            self.logger.warning(
                "find nan steering_percentage: %s" % str(self.chassis))
            return
        carspeed = self.chassis.speed_mps
        caracceleration = self.localization.pose.linear_acceleration_vrf.y

        speed_epsilon = 1e-9
        if abs(self.prev_carspeed) < speed_epsilon \
            and abs(carspeed) < speed_epsilon:
            caracceleration = 0.0

        carsteer = self.chassis.steering_percentage
        curvature = math.tan(math.radians(carsteer / 100 * 470) / 16) / 2.85
        if abs(carspeed) >= speed_epsilon:
            carcurvature_change_rate = (curvature - self.carcurvature) / (
                carspeed * 0.01)
        else:
            carcurvature_change_rate = 0.0
        self.carcurvature = curvature
        cartime = self.localization.header.timestamp_sec
        cargear = self.chassis.gear_location

        if abs(carspeed) >= speed_epsilon:
            if self.startmoving == False:
                self.logger.info(
                    "carspeed !=0 and startmoving is False, Start Recording")
            self.startmoving = True

        if self.startmoving:
            self.cars = self.cars + carspeed * 0.01 #("x,y,heading,real_speed,control_time,steering,lateral_error,heading_error\n")#plan_x,plan_y
            # self.write(
            #     "%s, %s, %s, %s, %s, %s, %s, %.4f, %s, %s, %s, %s, %s, %s\n" %
            #     (carx, cary, carz, carspeed, caracceleration, self.carcurvature,
            #      carcurvature_change_rate, cartime, cartheta, cargear,
            #      self.cars, self.chassis.throttle_percentage,
            #      self.chassis.brake_percentage,
            #      self.chassis.steering_percentage))
            # self.write(
            #     "%s, %s, %.4f, %s, %.4f,%.4f,%.4f,%.4f\n" %
            #     (carx, cary,  self.control_cmd.debug.simple_lat_debug.heading, carspeed, self.control_time,
            #      self.control_cmd.debug.simple_lat_debug.steer_angle, self.lateral_error, self.heading_error))
            self.write(
                "%.4f, %.4f, %.4f,%.4f,%.4f,%.4f\n" %
                (self.control_time, self.control_cmd.debug.simple_lat_debug.steer_angle
                 , self.control_cmd.debug.simple_lat_debug.steer_angle_feedback, self.control_cmd.debug.simple_lat_debug.steer_angle_feedforward,
                 self.lateral_error, self.heading_error))
            self.logger.debug(
                "started moving and write data at time %s" % cartime)
        else:
            self.logger.debug("not start moving, do not write data to file")

        self.prev_carspeed = carspeed

    def shutdown(self):
        """
        shutdown rosnode
        """
        self.terminating = True
        self.logger.info("Shutting Down...")
        self.logger.info("file is written into %s" % self.record_file)
        self.file_handler.close()
        rospy.sleep(0.1)


def main(argv):
    """
    Main rosnode
    """
    rospy.init_node('record_plot', anonymous=True)
    # rospy.init_node('rtk_recorder', anonymous=True)

    argv = FLAGS(argv)
    log_dir = os.path.dirname(os.path.abspath(__file__)) + "/../../../data/log/"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    Logger.config(
        log_file=log_dir + "record_plot.log",
        use_stdout=True,
        log_level=logging.DEBUG)
    print("runtime log is in %s%s" % (log_dir, "record_plot.log"))
    record_file = log_dir + "/lqr_steer_angle.csv"
    recorder = RecordPlot(record_file)
    atexit.register(recorder.shutdown)
    rospy.Subscriber('/apollo/canbus/chassis', chassis_pb2.Chassis,
                     recorder.chassis_callback)

    rospy.Subscriber('/apollo/localization/pose',
                     localization_pb2.LocalizationEstimate,
                     recorder.localization_callback)

    rospy.Subscriber('/apollo/control',
                     control_cmd_pb2.ControlCommand,
                     recorder.control_callback)

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
