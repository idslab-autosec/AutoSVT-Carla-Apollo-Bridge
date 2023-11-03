import carla
from carla import Transform,Location,Rotation,Vector3D,VehicleControl
import argparse
import time

def parse_arguments():
    parser = argparse.ArgumentParser(description='Move ego vehicle to (x, y, z, pitch, yaw, roll).')
    parser.add_argument('--host',help="Carla server ip", type=str, default="127.0.0.1")
    parser.add_argument('--port',help="Carla server port", type=int, default=2000)

    parser.add_argument('-x', '--x', help='x', type=float, default=0)
    parser.add_argument('-y','--y',help='y',type=int,default=0)
    parser.add_argument('-z', '--z', help='z', type=float, default=0)
    parser.add_argument('--pitch', help='pitch in degree', type=float, default=0)
    parser.add_argument('--yaw', help='yaw in degree, (North->-90°, East->0°)', type=float, default=0)
    parser.add_argument('--roll', help='roll in degree', type=float, default=0)
    arguments = parser.parse_args()
    return arguments  

def get_ego_vehicle(world):
    world.wait_for_tick()
    actor_list = world.get_actors()
    for actor in actor_list:
        try:
            if "hero" == actor.attributes['role_name'] or "ego_vehicle" == actor.attributes['role_name']:
                return actor
        except:
            pass
    return None

def main():
    args = parse_arguments()
    client = carla.Client(args.host, args.port)
    client.set_timeout(3.0)

    world = client.get_world()
    world.wait_for_tick()
    ego_vehicle = get_ego_vehicle(world)          
    vehicle_control = VehicleControl()
    vehicle_control.brake = 1.00
    ego_vehicle.apply_control(vehicle_control)
    ego_vehicle.set_target_velocity(Vector3D(0,0,0))

    loc = carla.Location(x=args.x, y=args.y, z=args.z)
    rot = carla.Rotation(pitch=args.pitch, yaw=args.yaw, roll=args.roll)
    ego_vehicle.set_transform(carla.Transform(loc, rot))

if __name__ == '__main__':
    main()
