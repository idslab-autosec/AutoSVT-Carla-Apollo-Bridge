import carla
import argparse

def parse_arguments():
    parser = argparse.ArgumentParser(description='Change weather of carla world')
    parser.add_argument('--host',help="Carla server host ip", type=str, default="127.0.0.1")
    parser.add_argument('-f', '--fog', help='Fog concentration or thickness. It only affects the RGB camera sensor. Values range from 0 to 100.', type=float, default=0.0)
    parser.add_argument('-c','--cloudiness', help="Values range from 0 to 100, being 0 a clear sky and 100 one completely covered with clouds.", type=float, default=10.0)
    parser.add_argument('-w', '--wetness', help="Wetness intensity. It only affects the RGB camera sensor. Values range from 0 to 100.", type=float, default=0.0)
    parser.add_argument('--fog-falloff', help="Density of the fog (as in specific mass) from 0 to infinity. The bigger the value, the more dense and heavy it will be, and the fog will reach smaller heights.If the value is 0, the fog will be lighter than air, and will cover the whole scene.A value of 1 is approximately as dense as the air, and reaches normal-sized buildings.For values greater than 5, the air will be so dense that it will be compressed on ground level.",
                        type=float, default=0.1)
    parser.add_argument('-i', '--scattering_intensity', help="Controls how much the light will contribute to volumetric fog. When set to 0, there is no contribution.", type=float, default=1.0)
    # parser.add_argument('-d', '--dust', help="Determines the strength of the dust storm weather. Values range from 0 to 100.", type=float, default=0.0)
    parser.add_argument('-s', '--sun-altitude-angle', help="90 is midday, -90 is midnight.", type=float, default=90.0)
    parser.add_argument('-r', '--rain', help="Rain intensity values range from 0 to 100, being 0 none at all and 100 a heavy rain.", type=float, default=0.0)
    arguments = parser.parse_args()
    return arguments

def main():
    args = parse_arguments()
    client = carla.Client(args.host, 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    weather = world.get_weather()
    world.wait_for_tick()

    weather.fog_density = args.fog
    weather.cloudiness = args.cloudiness
    weather.wetness = args.wetness
    weather.fog_falloff = args.fog_falloff
    weather.scattering_intensity = args.scattering_intensity
    # weather.dust_storm = args.dust
    weather.sun_altitude_angle = args.sun_altitude_angle
    weather.percipitation = args.rain

    world.set_weather(weather)
    world.wait_for_tick()

if __name__ == '__main__':
    main()