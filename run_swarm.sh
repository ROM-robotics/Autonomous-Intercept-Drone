#!/bin/bash

# =======================================================
# PX4 双机仿真启动脚本
# Drone 1: gz_x500_depth (带深度相机, Instance 1)
# Drone 2: gz_x500       (普通版,     Instance 2)
# 世界: grass_world
# =======================================================

set -e

# -------------------------------------------------------
# 0. 检查工作目录 & 依赖工具
# -------------------------------------------------------
if [ ! -f "build/px4_sitl_default/bin/px4" ]; then
    echo "❌ 错误：请在 PX4-Autopilot 根目录下运行此脚本"
    echo "   当前目录: $(pwd)"
    exit 1
fi

if ! command -v rotatelogs &>/dev/null; then
    echo "❌ 错误：未找到 rotatelogs，请安装 apache2-utils"
    echo "   sudo apt install apache2-utils"
    exit 1
fi

# 日志轮转配置：每个日志最大 5MB，保留最近 2 个文件
LOG_MAX_SIZE="5M"
LOG_MAX_FILES=2

# -------------------------------------------------------
# 1. 彻底清理残留进程
# -------------------------------------------------------
echo "🧹 正在清理残留进程..."
killall -9 px4 gz-sim gz-server gz-client MicroXRCEAgent parameter_bridge 2>/dev/null || true
sleep 2

# 清理旧日志
if ls *.log 1>/dev/null 2>&1; then
    rm -f ./*.log
    echo "   -> 已清除旧的运行日志 (*.log)"
fi

# 清理 PX4 仿真缓存
PX4_LOG_DIR="build/px4_sitl_default/tmp/rootfs/log"
if [ -d "$PX4_LOG_DIR" ]; then
    rm -rf "${PX4_LOG_DIR:?}"/*
    echo "   -> 已清理 PX4 仿真缓存日志"
fi

# -------------------------------------------------------
# 2. 启动 MicroXRCEAgent (ROS 2 通信桥)
# -------------------------------------------------------
echo ""
echo "📡 正在启动 MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 2>&1 | rotatelogs -n $LOG_MAX_FILES agent.log $LOG_MAX_SIZE &
PID_AGENT=$!
echo "   -> Agent PID: $PID_AGENT (日志: agent.log, 最大 ${LOG_MAX_SIZE} x${LOG_MAX_FILES})"
sleep 2

# -------------------------------------------------------
# 3. 启动第一架无人机 (带深度相机, Instance 1)
#    同时拉起 Gazebo 仿真服务器 + grass_world
# -------------------------------------------------------
echo ""
echo "🚀 正在启动 Drone 1 (gz_x500_depth, Instance 1)..."
PX4_GZ_WORLD=grass_world \
PX4_SYS_AUTOSTART=4002 \
PX4_GZ_MODEL_POSE="0,0" \
PX4_SIM_MODEL=gz_x500_depth \
PX4_PARAM_EKF2_HGT_REF=2 \
    ./build/px4_sitl_default/bin/px4 -i 1 2>&1 | rotatelogs -n $LOG_MAX_FILES drone1.log $LOG_MAX_SIZE &
PID1=$!
echo "   -> Drone 1 PID: $PID1 (日志: drone1.log, 最大 ${LOG_MAX_SIZE} x${LOG_MAX_FILES})"

echo ""
echo "⏳ 等待 Gazebo 世界加载 (15 秒)..."
sleep 15

	# -------------------------------------------------------
	# 4. 启动 ROS 2 相机桥接 (YAML 内嵌，直接映射到 /camera/image)
	# -------------------------------------------------------
	echo ""
	echo "🌉 正在启动 ROS 2 相机桥接..."

	cat > /tmp/camera_bridge.yaml << 'YEOF'
- ros_topic_name: "/camera/image"
  gz_topic_name: "/world/grass_world/model/x500_depth_1/link/camera_link/sensor/IMX214/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
YEOF

	ros2 run ros_gz_bridge parameter_bridge \
	    --ros-args -p config_file:="/tmp/camera_bridge.yaml" \
	    2>&1 | rotatelogs -n $LOG_MAX_FILES bridge.log $LOG_MAX_SIZE &
	PID_BRIDGE=$!
	echo "   -> Bridge PID: $PID_BRIDGE (bridge.log)"
	echo "   -> /camera/image"
	sleep 2
echo ""
echo "🚀 正在启动 Drone 2 (gz_x500, Instance 2)..."
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=grass_world \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="20,0" \
PX4_SIM_MODEL=gz_x500 \
    ./build/px4_sitl_default/bin/px4 -i 2 2>&1 | rotatelogs -n $LOG_MAX_FILES drone2.log $LOG_MAX_SIZE &
PID2=$!
echo "   -> Drone 2 PID: $PID2 (日志: drone2.log, 最大 ${LOG_MAX_SIZE} x${LOG_MAX_FILES})"
sleep 3

# -------------------------------------------------------
# 6. 启动摘要
# -------------------------------------------------------
echo ""
echo "✅ 所有系统启动完成！"
echo "================================================"
echo "  MicroXRCEAgent : PID $PID_AGENT  (port 8888)"
echo "  Drone 1 (相机)  : PID $PID1      Instance 1  UXRCE_DDS_KEY=2  /px4_1/"
echo "  Drone 2 (普通)  : PID $PID2      Instance 2  UXRCE_DDS_KEY=3  /px4_2/"
echo "  Camera Bridge  : PID $PID_BRIDGE"
echo "------------------------------------------------"
echo "  日志限制        : 每个最大 ${LOG_MAX_SIZE}，保留 ${LOG_MAX_FILES} 个文件"
echo "  相机话题        : /camera/image"
echo "  世界地图        : grass_world"
echo "  Drone 1 位置   : (0,  0)"
echo "  Drone 2 位置   : (0, 20)"
echo "================================================"
echo ""
echo "  验证命令:"
echo "  ros2 topic list | grep camera"
echo "  ros2 topic hz /camera/image"
echo "  ros2 topic list | grep px4"
echo ""
echo "按 Ctrl+C 关闭所有仿真进程"

# -------------------------------------------------------
# 7. 退出时清理所有子进程
# -------------------------------------------------------
cleanup() {
    echo ""
    echo "🛑 正在关闭所有进程..."
    kill "$PID1" "$PID2" "$PID_AGENT" "$PID_BRIDGE" 2>/dev/null || true
    sleep 1
    killall -9 px4 gz-sim gz-server gz-client MicroXRCEAgent parameter_bridge 2>/dev/null || true
    echo "✅ 清理完成"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 等待主进程退出
wait "$PID1" "$PID2" "$PID_AGENT" "$PID_BRIDGE"
