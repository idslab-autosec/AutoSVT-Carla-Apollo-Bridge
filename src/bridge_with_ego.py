#!/usr/bin/env python
#
# Copyright (c) 2022 HIT-IDS
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Class that handle communication between CARLA and Cyber
"""
import sys
import argparse
import logging
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/../")
import carla
import json
import datetime
import threading

import pygame
import numpy as np

from custom_bridge.custom_bridge import CustomCarlaCyberBridge

import yaml


class Display(threading.Thread):
    def __init__(self, world, objects_file_object):
        threading.Thread.__init__(self)
        self.world = world
        self.objects_file_object = objects_file_object

    def run(self):
        pygame.init()
        pygame.font.init()
        try:
            car = get_ego_vehicle(self.world, self.objects_file_object)
            camera_manager = CameraManager(self.world, car)
            # imu_manager = ImuManager(self.world, car)
            image_w = camera_manager.camera_bp.get_attribute("image_size_x").as_int()
            image_h = camera_manager.camera_bp.get_attribute("image_size_y").as_int()
            display = pygame.display.set_mode((image_w, image_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
            display.fill((0, 0, 0))
            clock = pygame.time.Clock()
            while True:
                # print("yaw:{}".format(car.get_transform().rotation))
                clock.tick_busy_loop(60)
                self.world.wait_for_tick()
                camera_manager.render(display)
                pygame.display.flip()
        finally:
            pygame.quit


class CameraManager(object):
    def __init__(self, world, car):
        self.surface = None
        self.world = world
        self.ego_vehicle = car
        blueprint_library = world.get_blueprint_library()
        self.camera_bp = blueprint_library.find('sensor.camera.rgb')
        # self.camera_bp.set_attribute('image_size_x', '1920')
        # self.camera_bp.set_attribute('image_size_y', '1080')
        self.camera_bp.set_attribute('image_size_x', '800')
        self.camera_bp.set_attribute('image_size_y', '600')
        # self.camera_bp.set_attribute('fov', '110')
        camera_transform = carla.Transform(carla.Location(x=-5, y=0, z=3), carla.Rotation(pitch=-20, yaw=0, roll=0))
        self.camera = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.ego_vehicle)
        # set the callback function
        self.camera.listen(lambda image: self._parse_image(image))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    def _parse_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))


def get_ego_vehicle(world, objects_file_object):
    for actor in objects_file_object["objects"]:
        actor_id = actor["id"]
        # if actor_id == "ego_vehicle":
        if actor_id == "hero":
            ego_vehicle_type = actor["type"]
            break

    ego_vehicle = None
    while ego_vehicle is None:
        actor_list = world.get_actors()
        for actor in actor_list:
            if actor.type_id == ego_vehicle_type:
                ego_vehicle = actor
                break
    return ego_vehicle


def main(args=None):
    """
    main function for carla simulator Cyber bridge
    maintaining the communication client and the CarlaBridge object
    """

    arg_parser = argparse.ArgumentParser(
        description='CARLA Auto Control Client')
    arg_parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    arg_parser.add_argument(
        '--host',
        metavar='H',
        default='172.17.0.1',
        help='IP of the host server (default: 172.17.0.1)')
    arg_parser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    arg_parser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    arg_parser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    arg_parser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    arg_parser.add_argument(
        '-m', '--map',
        metavar='M',
        default='',
        help='Map of the world (default: "Town03")')
    args = arg_parser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    # log_level = logging.WARNING

    logging.basicConfig(
        format='%(asctime)s,%(msecs)03d %(levelname)-3s [%(threadName)s] [%(filename)s:%(lineno)d] %(message)s',
        datefmt='%Y-%m-%d:%H:%M:%S',
        level=log_level)

    world = None
    try:
        # System config
        config_file = os.path.dirname(os.path.realpath(__file__)) + "/config/settings.yaml"
        parameters = yaml.safe_load(open(config_file))['carla']

        logging.info('Listening to server %s:%s', args.host, args.port)
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        if args.map != '':
            carla_map = args.map
        else:
            carla_map = parameters["map"]

        logging.info("Loading map %s", carla_map)
        client.load_world(carla_map)
        world = client.get_world()
        original_settings = world.get_settings()
        traffic_manager = client.get_trafficmanager()

        carla_settings = world.get_settings()
        carla_settings.synchronous_mode = parameters["synchronous_mode"]
        carla_settings.fixed_delta_seconds = parameters["fixed_delta_seconds"]
        world.apply_settings(carla_settings)
        if carla_settings.synchronous_mode:
            traffic_manager.set_synchronous_mode(True)

        world.recording_enabled = False
        if world.recording_enabled:
            filename = 'record_' + str(datetime.datetime.now()) + '.log'
            client.start_recorder(filename, True)
        weather = world.get_weather()
        weather.sun_azimuth_angle = 0.0
        weather.sun_altitude_angle = 90.0
        weather.precipitation = 0.0
        world.set_weather(weather)

        # Object config
        objects_file = os.path.dirname(os.path.realpath(__file__)) + '/config/objects.json'
        with open(objects_file) as handle:
            objects_file_object = json.loads(handle.read())

        # Start display camera actor
        display = Display(world, objects_file_object)
        display.setDaemon(True)
        display.start()

        # Spawn actors and start bridge
        bridge = CustomCarlaCyberBridge(world, parameters, objects_file_object)
        bridge.run()
    except Exception as e:
        print (e)
    finally:
        if original_settings:
            world.apply_settings(original_settings)
        if world and world.recording_enabled:
            client.stop_recorder()
        # if world is not None:
        #     world.destroy()
        pygame.quit()


if __name__ == "__main__":
    main()
