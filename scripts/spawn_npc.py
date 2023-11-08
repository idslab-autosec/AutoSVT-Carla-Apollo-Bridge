import carla
import math
import argparse

from move_ego import get_ego_vehicle


def parse_arguments():
    parser = argparse.ArgumentParser(description="Spawn npc")
    parser.add_argument(
        "--host", help="Carla server host ip", type=str, default="127.0.0.1"
    )
    parser.add_argument(
        "--port", help="Carla server port", type=int, default=2000
    )
    parser.add_argument(
        "-f", "--front", help="front distance (m)", type=float, default=10
    )
    parser.add_argument(
        "-r", "--right", help="right distance (m)", type=float, default=0
    )
    parser.add_argument(
        "-y", "--yaw", help="delta yaw of npc (degree)", type=float, default=0
    )
    parser.add_argument(
        "-t",
        "--type",
        help="Filter of npc vehicle",
        type=str,
        default="vehicle.tesla.model3",
    )
    arguments = parser.parse_args()
    return arguments


# Frequently used npc_type: ("vehicle.tesla.model3", "vehicle.ford.ambulance",
# "vehicle.volkswagen.t2", "vehicle.tesla.cybertruck", "vehicle.nissan.patrol_2021")
def spawn_npc(world, ego, args):
    blueprint_library = world.get_blueprint_library()
    npc_bp = blueprint_library.filter(args.type)[0]
    spawn_map = world.get_map()
    spawn_points = spawn_map.get_spawn_points()
    spawn_point = spawn_points[1]
    # print("npc_bp: {}, spawn_point: {}".format(npc_bp, spawn_point))
    npc = world.try_spawn_actor(npc_bp, spawn_point)
    # npc = world.spawn_actor(npc_bp, spawn_point)
    # print("npc: {}".format(npc))

    ego_tf = ego.get_transform()
    yaw = math.radians(ego_tf.rotation.yaw)
    ego_tf.rotation.yaw += args.yaw
    # print("yaw = {}, sin(yaw) = {}, cos(yaw) = {}".format(yaw, math.sin(yaw), math.cos(yaw)))
    npc_x = ego_tf.location.x + args.front * math.cos(yaw) + args.right * math.sin(yaw)
    npc_y = ego_tf.location.y + args.front * math.sin(yaw) + args.right * math.cos(yaw)
    npc_z = ego_tf.location.z + 0.5
    target_point = carla.Transform(carla.Location(npc_x, npc_y, npc_z), ego_tf.rotation)
    world.wait_for_tick()
    # print("target_point = {}".format(target_point))
    npc.set_transform(target_point)
    print("spawn npc:{}".format(target_point))
    world.wait_for_tick()
    # print("Ego vehicle location: {}".format(ego_tf.location))
    # print("Spawn npc at {}.".format(npc.get_transform().location))
    return npc


# def get_ego_vehicle(world):
#     actor_list = world.get_actors()
#     for actor in actor_list:
#         # print(actor.attributes)
#         try:
#             if "hero" == actor.attributes['role_name']:
#                 return actor
#         except:
#             pass
#     return False


def main():
    args = parse_arguments()
    client = carla.Client(args.host, args.port)
    client.set_timeout(3.0)
    world = client.get_world()
    world.wait_for_tick()
    ego = get_ego_vehicle(world)
    npc = spawn_npc(world, ego, args)

    try:
        while True:
            world.wait_for_tick()
    except KeyboardInterrupt:
        client.apply_batch([carla.command.DestroyActor(npc)])


if __name__ == "__main__":
    main()
