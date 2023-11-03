#!/usr/bin/env python

#
# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla Radar
"""

import numpy as np

from carla_cyber_bridge.sensor import Sensor

from modules.common_msgs.sensor_msgs.conti_radar_pb2 import ContiRadar, ContiRadarObs

class Radar(Sensor):

    """
    Actor implementation details of Carla RADAR
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_cyber_bridge.Parent
        :param relative_spawn_pose: the spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Radar, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    relative_spawn_pose=relative_spawn_pose,
                                    node=node,
                                    carla_actor=carla_actor,
                                    synchronous_mode=synchronous_mode)

        self.radar_writer = node.new_writer(self.get_topic_prefix(), ContiRadar, qos_depth=10)
        self.listen()

    def destroy(self):
        super(Radar, self).destroy()

    def get_topic_prefix(self):
        """
        get the topic name of the current entity.

        :return: the final topic name of this object
        :rtype: string
        """
        return "/apollo/sensor/" + self.name

    def sensor_data_updated(self, carla_radar_measurement):
        """
        Function to transform the a received Radar measurement into a ROS message
        :param carla_radar_measurement: carla Radar measurement object
        :type carla_radar_measurement: carla.RadarMeasurement
        """
        conti_radar = ContiRadar()
        conti_radar.header.CopyFrom(self.parent.get_msg_header(timestamp=carla_radar_measurement.timestamp))
        for detection in carla_radar_measurement:
            conti_radar_obs = ContiRadarObs()
            conti_radar_obs.lateral_dist = detection.depth * np.cos(detection.azimuth) * np.cos(-detection.altitude)
            conti_radar_obs.longitude_dist = detection.depth * np.sin(-detection.azimuth) * np.cos(detection.altitude)
            conti_radar_obs.lateral_vel = detection.velocity * np.sin(-detection.azimuth) * np.cos(detection.altitude)
            conti_radar_obs.longitude_vel = detection.velocity * np.cos(detection.azimuth) * np.cos(-detection.altitude)
            conti_radar.contiobs.append(conti_radar_obs)

        # frame_id = self.carla_actor.attributes['role_name']
        # conti_radar.header.frame_id = frame_id
        self.radar_writer.write(conti_radar)
