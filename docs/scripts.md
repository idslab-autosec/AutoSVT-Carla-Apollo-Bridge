# Scripts
This project provides some Python scripts for Carla, making testing more convenient. All scripts can be found in `scripts/`.
## 1. set_weather.py
One of the most significant features of this project is the simulation of the impact of fog on LiDAR. Weather parameters, including fog, can be configured using `set_weather.py`.
```bash
cd scripts
# Set the fog density to 50 and the sun altitude angle to -90
python set_weather.py -f 50 -s -90
```

## 2. move_ego.py
This script allows for the movement of the ego vehicle to specific coordinates on the map (Carla coordinate system). You can find the coordinates on the map in Apollo DreamView's Route Editing. However, please note that **the y-coordinates in Apollo are the opposite of the y-coordinates in Carla**.
```bash
cd scripts
python move_ego.py -x -31.08 -y 134.57 --yaw=0
```

## 3. spawn_npc.py
This script is used to generate an NPC vehicle near the ego vehicle.
```bash
cd scripts
# Generate a Tesla Model 3 vehicle 
# 20 meters ahead and 2 meters to the right of the ego vehicle.
python spawn_npc.py -f 20 -r 2 -t "vehicle.tesla.model3"
```

## 4. generate_traffic.py
This script is directly copied from the examples in the Carla open-source code and is used to generate dynamic traffic flow.
```bash
cd scripts
# Randomly spawn 40 vehicles and 10 pedestrians.
python generate_traffic.py -n 40 -w 10 --hybrid
```