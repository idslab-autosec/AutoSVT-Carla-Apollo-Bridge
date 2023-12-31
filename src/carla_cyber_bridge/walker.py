#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla pedestrians
"""

from carla import WalkerControl

from carla_cyber_bridge.traffic_participant import TrafficParticipant

from bridge.carla_bridge.carla_proto.proto.carla_walker_control_pb2 import CarlaWalkerControl
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacle


class Walker(TrafficParticipant):

    """
    Actor implementation details for pedestrians
    """

    def __init__(self, uid, name, parent, node, carla_actor):
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
        :param carla_actor: carla walker actor object
        :type carla_actor: carla.Walker
        """
        super(Walker, self).__init__(uid=uid,
                                     name=name,
                                     parent=parent,
                                     node=node,
                                     carla_actor=carla_actor)

        self.control_reader = self.node.new_reader(
            self.get_topic_prefix() + "/walker_control_cmd",
            CarlaWalkerControl,
            self.control_command_updated)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate CyberRT readers
        Finally forward call to super class.

        :return:
        """
        super(Walker, self).destroy()

    def control_command_updated(self, cyber_walker_control):
        """
        Receive a CarlaWalkerControl msg and send to CARLA
        This function gets called whenever a CyberRT message is received via
        '/carla/<role name>/walker_control_cmd' topic.
        The received CyberRT message is converted into carla.WalkerControl command and
        sent to CARLA.
        :param cyber_walker_control: current walker control input received via CyberRT
        :type self.info.output: carla_cyber_bridge.msg.CarlaWalkerControl
        :return:
        """
        walker_control = WalkerControl()
        walker_control.direction.x = cyber_walker_control.direction.x
        walker_control.direction.y = -cyber_walker_control.direction.y
        walker_control.direction.z = cyber_walker_control.direction.z
        walker_control.speed = cyber_walker_control.speed
        walker_control.jump = cyber_walker_control.jump
        self.carla_actor.apply_control(walker_control)

    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return PerceptionObstacle.PEDESTRIAN
