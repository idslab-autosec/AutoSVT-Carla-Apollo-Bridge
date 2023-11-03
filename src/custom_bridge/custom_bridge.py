#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Cyberbridge class:

Class that handle communication between CARLA and Cyber
"""
import itertools
import sys, os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/../../")
import logging
import threading
from pycyber import cyber,cyber_time
from cyber_compatibility.node import CompatibleNode
import random
import carla
from transforms3d.euler import euler2mat, quat2euler, euler2quat
import math
from modules.common_msgs.localization_msgs.pose_pb2 import Pose
# from modules.cyber.proto.clock_pb2 import Time, Clock
from modules.cyber.proto.clock_pb2 import Clock
from carla_cyber_bridge.actor_factory import ActorFactory
from threading import Thread, Lock, Event
secure_random = random.SystemRandom()


class CustomCarlaCyberBridge(CompatibleNode):
    """
    Carla Cyber bridge
    """

    def __init__(self, world, parameters, objects_file_object):
        """
        Constructor

        """
        super(CustomCarlaCyberBridge, self).__init__("cyber_bridge_node")

        self.world = world
        self.parameters = parameters
        self.objects_file_object = objects_file_object

        self.spawn_points = world.get_map().get_spawn_points()
        self.blueprint_lib = world.get_blueprint_library()
        self.last_run_timestamp = 0.0
        self.actor_list = []

        self.update_lock = threading.Lock()

        self.sync_mode = self.parameters["synchronous_mode"]
        self.actor_factory = ActorFactory(self, world, self.sync_mode)
        self.clock_writer = self.new_writer('/clock', Clock, 10)
        # id generator for pseudo sensors
        self.id_gen = itertools.count(10000)

        self.shutdown = Event()

    def run(self):
        # Spawn all objects in carla.
        self.spawn_actors(self.objects_file_object["objects"], None)
        if self.sync_mode:
            logging.info("Running bridge in synchronous mode")
            thread = Thread(target=self._synchronous_mode_update)
            thread.setDaemon(True)
            thread.start()
        else:
            logging.info("Running bridge in asynchronous mode")
            # Register callback to update actors in carla.
            self.world.on_tick(self._do_when_time_tick)

        # Spin thread.
        self.spin()
        self.destroy()

    def spawn_actors(self, actors, attach_actor):
        for actor in actors:
            logging.debug("for actor in actors %s",actor)
            if "pseudo" in str(actor['type']):
                if attach_actor is not None:
                    attach_id = attach_actor.id
                else:
                    attach_id = 0
                actor_object = self.actor_factory._create_object(
                    next(self.id_gen),
                    actor['type'],
                    actor['id'],
                    attach_id,
                    None)
            else:
                type_pos_0 = actor['type'].split('.')[0]
                if type_pos_0 == "vehicle":
                    carla_actor = self.spawn_actor(actor, None)
                    if actor["sensors"]:
                        self.spawn_actors(actor["sensors"], carla_actor)
                else:
                    carla_actor = self.spawn_actor(actor, attach_actor)
                if carla_actor  != None:
                    actor_object = self.actor_factory._create_object_from_actor(carla_actor)
            if actor_object is not None:
                self.actor_list.append(actor_object)

    def spawn_actor(self, actor, attach_actor):
        """
        spawns an actor in carla
        """
        actor_name = actor["type"] + "/" + actor["id"]
        logging.info("Spawning actor %s.", actor_name)
        blueprint = self.blueprint_lib.find(str(actor["type"]))
        blueprint.set_attribute('role_name', str(actor["id"]))
        # Add other attributes.
        if "other_attr" in actor:
            other_attr = actor.pop("other_attr")
            for key, value in other_attr.items():
                logging.debug("Set attribute[%s] = %s.", key, value, )
                blueprint.set_attribute(str(key), str(value))

        # Process transform.
        if "spawn_point" in actor:
            spawn_point = actor.pop("spawn_point")
            cyber_pose = self.create_spawn_point(
                spawn_point.pop("x", 0.0),
                spawn_point.pop("y", 0.0),
                spawn_point.pop("z", 0.0),
                spawn_point.pop("roll", 0.0),
                spawn_point.pop("pitch", 0.0),
                spawn_point.pop("yaw", 0.0)
            )
            transform = carla.Transform(
                self.cyber_point_to_carla_location(cyber_pose.position),
                self.cyber_quaternion_to_carla_rotation(cyber_pose.orientation))
        else:
            if "vehicle" in str(actor['type']):
                transform = secure_random.choice(self.spawn_points)
            else:
                cyber_pose = self.create_spawn_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                transform = carla.Transform(
                    self.cyber_point_to_carla_location(cyber_pose.position),
                    self.cyber_quaternion_to_carla_rotation(cyber_pose.orientation))

        # Spawn actor.
        if attach_actor:
            logging.info("Actor[%s] will be attached to %s", actor_name, attach_actor)
            carla_actor = self.world.try_spawn_actor(blueprint, transform, attach_actor)
        else:
            carla_actor = self.world.try_spawn_actor(blueprint, transform)

        logging.info("Spawn actor[%s] success.", carla_actor)
        return carla_actor
    
    def get_timestamp(self,sec):
        total = 0
        total = cyber_time.Time(float(sec)).to_nsec()
        return total
        # secs = total / 1000000000
        # nsecs = total - secs * 1000000000
        # print(secs)
        # print(nsecs)
        # return {'secs': int(secs), 'nsecs': int(nsecs)}
    
    def update_clock(self, carla_timestamp):
        """
        perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if cyber.ok():
            self.timestamp_nsecs = self.get_timestamp(carla_timestamp.elapsed_seconds) + 1
            self.clock_writer.write(Clock(nsecs=self.timestamp_nsecs))
            # self.clock_writer.write(Clock(clock=Time(secs=self.timestamp['secs'],
            #                                          nsecs=self.timestamp['nsecs'])))

    def _synchronous_mode_update(self):
        while not self.shutdown.is_set():
            frame = self.world.tick()
            carla_timestamp = self.world.get_snapshot()
            self.update_clock(carla_timestamp)
            self.update(carla_timestamp.frame, carla_timestamp.timestamp.elapsed_seconds)
            # self.update_clock(carla_timestamp)

    def _do_when_time_tick(self, carla_timestamp):
        """
        Do things when carla world.on_tick()

        """
        if not cyber.ok():
            return
        # print("_do_when_time_tick")
        if self.update_lock.acquire(False):
            if carla_timestamp.elapsed_seconds % 10 < 0.01:
                logging.info("Tick on %s, last_run_timesampe %s", carla_timestamp, self.last_run_timestamp)

            if self.last_run_timestamp < carla_timestamp.elapsed_seconds:
                self.last_run_timestamp = carla_timestamp.elapsed_seconds
                # can't do this since there's no 'sim time' in cyber
                # self._update_clock(carla_timestamp)
                self.update(carla_timestamp.frame,
                            carla_timestamp.timestamp.elapsed_seconds)
            self.update_lock.release()

    def update(self, frame_id, timestamp):
        """
        update all actors
        :return:
        """
        self.actor_factory.update_actor_states(frame_id, timestamp)

    def destroy(self):
        """
        Function (virtual) to destroy this object.

        Lock the update mutex.
        Remove all publisher.
        Finally forward call to super class.

        :return:
        """
        if not self.update_lock.acquire(False):
            logging.warning('Failed to acquire update lock')
        for actor in self.actor_list:
            if actor is not None:
                logging.info("destroy %s", actor)
                actor.destroy()
        cyber.shutdown()

    def create_spawn_point(self, x, y, z, roll, pitch, yaw):
        spawn_point = Pose()
        spawn_point.position.x = x
        spawn_point.position.y = y
        spawn_point.position.z = z
        quat = euler2quat(math.radians(roll), math.radians(pitch), math.radians(yaw))

        spawn_point.orientation.qx = quat[1]
        spawn_point.orientation.qy = quat[2]
        spawn_point.orientation.qz = quat[3]
        spawn_point.orientation.qw = quat[0]
        return spawn_point

    def cyber_point_to_carla_location(self, cyber_point):
        return carla.Location(cyber_point.x, -cyber_point.y, cyber_point.z)

    def cyber_quaternion_to_carla_rotation(self, cyber_quaternion):
        roll, pitch, yaw = quat2euler([cyber_quaternion.qw,
                                       cyber_quaternion.qx,
                                       cyber_quaternion.qy,
                                       cyber_quaternion.qz])
        return self.rpy_to_carla_rotation(roll, pitch, yaw)

    def rpy_to_carla_rotation(self, roll, pitch, yaw):
        return carla.Rotation(roll=math.degrees(roll),
                              pitch=-math.degrees(pitch),
                              yaw=-math.degrees(yaw))

    def _ego_vehicle_control_applied_callback(self, ego_vehicle_id):
        if not self.sync_mode or \
                not self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
            return
        with self._expected_ego_vehicle_control_command_ids_lock:
            if ego_vehicle_id in self._expected_ego_vehicle_control_command_ids:
                self._expected_ego_vehicle_control_command_ids.remove(
                    ego_vehicle_id)
            else:
                self.logwarn(
                    "Unexpected vehicle control command received from {}".format(ego_vehicle_id))
            if not self._expected_ego_vehicle_control_command_ids:
                self._all_vehicle_control_commands_received.set()
