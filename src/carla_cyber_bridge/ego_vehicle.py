#!/usr/bin/env python

#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""
import math
import os
import time
from transforms3d.euler import euler2mat, quat2euler, euler2quat
from scipy.spatial.transform import Rotation as R

from pycyber import cyber_time

import numpy as np
from carla import VehicleControl, Location

from carla_cyber_bridge.vehicle import Vehicle

from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate, LocalizationStatus
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.transform_msgs.transform_pb2 import TransformStamped, TransformStampeds
import logging
import csv
try:
    import queue
except ImportError:
    import Queue as queue
from threading import Thread


class RecordLog(Thread):
    def __init__(self, log_queue, log_file_path):
        Thread.__init__(self)
        self.log_queue = log_queue
        self.log_file_path = log_file_path

    def run(self):
        logging.info("Write log to file: %s", self.log_file_path)
        with open(self.log_file_path, 'w') as f:
            writer = csv.writer(f)
            while True:
                while not self.log_queue.empty():
                    log = self.log_queue.get()
                    writer.writerow(log)
                time.sleep(0.5)


class EgoVehicle(Vehicle):
    """
    Vehicle implementation details for the ego vehicle
    """

    def __init__(self, uid, name, parent, node, carla_actor, world, vehicle_control_applied_callback):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_cyber_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        super(EgoVehicle, self).__init__(uid=uid,
                                         name=name,
                                         parent=parent,
                                         node=node,
                                         carla_actor=carla_actor)
        self.world = world

        self.vehicle_info_writed = False
        self.vehicle_control_override = False
        self.vehicle_loc_set = False
        # self.vehicle_loc_set = True
        self.perfect_localization = False
        self._vehicle_control_applied_callback = vehicle_control_applied_callback

        self.vehicle_chassis_writer = node.new_writer(
            "/apollo/canbus/chassis",
            Chassis,
            qos_depth=10)
        # self.vehicle_info_writer = node.new_writer(
        #     "/apollo/vehicle_info",
        #     CarlaEgoVehicleInfo,
        #     qos_depth=10)
        
        # self.vehicle_pose_writer = node.new_writer(
        #     "/apollo/localization/pose",
        #     LocalizationEstimate,
        #     qos_depth=10)
        
        self.localization_status_writer = node.new_writer(
            "/apollo/localization/msf_status",
            LocalizationStatus,
            qos_depth=10)
        self.tf_writer = node.new_writer("/tf", TransformStampeds)

        self.control_reader = node.new_reader(
            "/apollo/control",
            ControlCommand,
            lambda data: self.control_command_updated(data, manual_override=False))


        self.send_vehicle_msgs_after_control = False


    def get_tf_msg(self,timestamp):
        pose = self.get_current_cyber_pose()

        tf_msg = TransformStamped()
        tf_msg.header.timestamp_sec = timestamp
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'localization'

        tf_msg.transform.translation.x = pose.position.x
        tf_msg.transform.translation.y = pose.position.y
        tf_msg.transform.translation.z = pose.position.z

        tf_msg.transform.rotation.qx = pose.orientation.qx
        tf_msg.transform.rotation.qy = pose.orientation.qy
        tf_msg.transform.rotation.qz = pose.orientation.qz
        tf_msg.transform.rotation.qw = pose.orientation.qw

        return tf_msg

    def send_vehicle_msgs(self, frame, timestamp):
        """
        send messages related to vehicle status

        :return:
        """
        vehicle_chassis = Chassis()
        vehicle_chassis.header.timestamp_sec = timestamp
        vehicle_chassis.header.frame_id = 'ego_vehicle'
        vehicle_chassis.engine_started = True
        vehicle_chassis.speed_mps = self.get_vehicle_speed_abs(self.carla_actor)
        vehicle_chassis.throttle_percentage = self.carla_actor.get_control().throttle * 100.0
        vehicle_chassis.brake_percentage = self.carla_actor.get_control().brake * 100.0
        vehicle_chassis.steering_percentage = -self.carla_actor.get_control().steer * 100.0
        vehicle_chassis.parking_brake = self.carla_actor.get_control().hand_brake
        vehicle_chassis.driving_mode = Chassis.DrivingMode.COMPLETE_AUTO_DRIVE
        self.vehicle_chassis_writer.write(vehicle_chassis)
       
        '''
        Mock localization estimate.
        '''
        if not self.vehicle_loc_set:
        
            self.vehicle_loc_set = True
            transform = self.carla_actor.get_transform()
            linear_vel = self.carla_actor.get_velocity()
            angular_vel = self.carla_actor.get_angular_velocity()
            accel = self.carla_actor.get_acceleration()
            localization_status = LocalizationStatus()
            localization_status.header.timestamp_sec = timestamp
            localization_status.fusion_status = 0  # OK = 0
            localization_status.measurement_time = timestamp
            localization_status.state_message = ""
            self.localization_status_writer.write(localization_status)

        tf_stampeds = TransformStampeds()
        tf_stampeds.transforms.append(self.get_tf_msg(timestamp=timestamp))
        self.tf_writer.write(tf_stampeds)
       
        
        
    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        self.send_vehicle_msgs(frame, timestamp)
        super(EgoVehicle, self).update(frame, timestamp)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS readers
        Finally forward call to super class.

        :return:
        """
        super(EgoVehicle, self).destroy()

    def control_command_override(self, enable):
        """
        Set the vehicle control mode according to cyber topic
        """
        self.vehicle_control_override = enable.data

    def control_command_updated(self, cyber_vehicle_control, manual_override):
        """
        Receive a ControlCommand msg and send to CARLA

        This function gets called whenever a ControlCommand is received.
        If the mode is valid (either normal or manual), the received ROS message is
        converted into carla.VehicleControl command and sent to CARLA.
        This bridge is not responsible for any restrictions on velocity or steering.
        It's just forwarding the ROS input to CARLA

        :param manual_override: manually override the vehicle control command
        :param cyber_vehicle_control: current vehicle control input received via ROS
        :type cyber_vehicle_control: ControlCommand
        :return:
        """
        if manual_override == self.vehicle_control_override:
            vehicle_control = VehicleControl()
            vehicle_control.throttle = cyber_vehicle_control.throttle / 100.0
            vehicle_control.brake = cyber_vehicle_control.brake / 100.0
            rate = cyber_vehicle_control.steering_rate / 100.0
            vehicle_control.steer = -cyber_vehicle_control.steering_target / 100.0
            # logging.debug("rate is %s, steer is %s", rate, -vehicle_control.steer)
            vehicle_control.hand_brake = cyber_vehicle_control.parking_brake
            vehicle_control.reverse = cyber_vehicle_control.gear_location == Chassis.GearPosition.GEAR_REVERSE

            self.carla_actor.apply_control(vehicle_control)
            self._vehicle_control_applied_callback(self.get_id())


    @staticmethod
    def get_vector_length_squared(carla_vector):
        """
        Calculate the squared length of a carla_vector
        :param carla_vector: the carla vector
        :type carla_vector: carla.Vector3D
        :return: squared vector length
        :rtype: float64
        """
        return carla_vector.x * carla_vector.x + \
               carla_vector.y * carla_vector.y + \
               carla_vector.z * carla_vector.z

    @staticmethod
    def get_vehicle_speed_squared(carla_vehicle):
        """
        Get the squared speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: squared speed of a carla vehicle [(m/s)^2]
        :rtype: float64
        """
        return EgoVehicle.get_vector_length_squared(carla_vehicle.get_velocity())

    @staticmethod
    def get_vehicle_speed_abs(carla_vehicle):
        """
        Get the absolute speed of a carla vehicle
        :param carla_vehicle: the carla vehicle
        :type carla_vehicle: carla.Vehicle
        :return: speed of a carla vehicle [m/s >= 0]
        :rtype: float64
        """
        speed = math.sqrt(EgoVehicle.get_vehicle_speed_squared(carla_vehicle))
        return speed
