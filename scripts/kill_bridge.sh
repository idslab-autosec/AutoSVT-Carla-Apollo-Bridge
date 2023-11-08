#!/bin/bash

pids=$(pgrep -f bridge_with_ego.py)

for pid in $pids
do
    echo "Killing process $pid..."
    kill -9 $pid
done

echo "All processes killed."
