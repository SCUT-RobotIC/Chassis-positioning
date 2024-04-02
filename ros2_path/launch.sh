#!/bin/bash

# 打开一个新的终端并运行server
gnome-terminal -- bash -c "cd ~/ros2_path; colcon build --packages-select show_path; . install/setup.bash; ros2 run show_path server; exec bash"

# 等待一段时间以确保server已经启动
sleep 3s

# 打开一个新的终端并运行client
gnome-terminal -- bash -c "cd ~/ros2_path; . install/setup.bash; ros2 run show_path client; exec bash"

# 等待一段时间以确保client已经启动
sleep 3s

# 打开一个新的终端并运行talker
gnome-terminal -- bash -c "cd ~/ros2_path; . install/setup.bash; ros2 run show_path talker; exec bash"