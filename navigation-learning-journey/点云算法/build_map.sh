#!/bin/bash

# 设置脚本在遇到错误时终止执行
set -e

# 配置网络接口
sudo ifconfig enp7s0 192.168.1.50

# 切换到脚本所在的目录
cd "$(dirname "$0")"

# 源catkin工作空间
source /home/zijin/catkin_livox_ros_driver2/devel/setup.bash

# 启动livox_ros_driver2节点
roslaunch livox_ros_driver2 msg_MID360.launch &
PID_LIVOX=$!
echo "Livox ROS Driver launched with PID: $PID_LIVOX"

# 等待3秒钟以确保第一个节点已经启动
echo "Waiting for 3 seconds to ensure Livox ROS Driver has started..."
sleep 4

# 启动fast_lio节点
echo "Starting Fast Lio..."
roslaunch fast_lio_localization sentry_build_map.launch  &
PID_FAST_LIO=$!
echo "Fast Lio launched with PID: $PID_FAST_LIO"

# 将节点放到后台运行，并等待它们完成
echo "Both nodes are running in the background."

# RViz节点的名称
RVIZ_NODE_NAME="rviz"

# 暂停作业
sleep 3
echo "Press Ctrl+C to pause the script..."
ctrl_z_and_bg=true
trap 'echo "Script paused"; ctrl_c_and_bg=false;' SIGTSTP
while $ctrl_c_and_bg; do
    sleep 1
done
trap - SIGTSTP

# 恢复作业
fg