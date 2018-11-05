#!/bin/bash

killbg() {
        for p in "${pids[@]}" ; do
                kill "$p";
        done
}
trap killbg EXIT
pids=()
roscore &
pids+=($!)
read -n 1 -s
cd ~/catkin_ws/
rosrun stage_ros stageros src/robots_exercise2/stage/pioneer_socs.world &
pids+=($!)
read -n 1 -s
rosrun map_server map_server src/robots_exercise2/stage/map.yaml &
pids+=($!)
read -n 1 -s
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map my_frame 100 &
pids+=($!)
read -n 1 -s
rosrun rviz rviz -d ~/.rviz/pose_display.rviz
pids+=($!)
read -n 1 -s
rosrun pf_localisation node.py &
pids+=($!)
