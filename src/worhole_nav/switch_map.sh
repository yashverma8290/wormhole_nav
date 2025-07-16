#!/bin/bash

MAP_FILE=$1

if [ -z "$MAP_FILE" ]; then
  echo "Usage: ./switch_map.sh <map_file.yaml>"
  exit 1
fi

echo "Switching to map: $MAP_FILE"

ros2 lifecycle set map_server shutdown
ros2 lifecycle set amcl shutdown
sleep 2

ros2 launch nav2_bringup localization_launch.py map:=$MAP_FILE use_sim_time:=false &

