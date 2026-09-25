#!/bin/bash
# =======================================================
# BC 数据采集启动脚本
# 依次启动: 仿真 → 目标机 → 检测 → PNG拦截 → 录制
#
# 用法:
#   ./launch_bc_collection.sh [mode]
#
#   mode: circle(默认) / sinusoidal / random_walk
#
# 示例:
#   ./launch_bc_collection.sh circle
#   ./launch_bc_collection.sh sinusoidal
# =======================================================
set -e

MODE="${1:-circle}"
OUT_DIR="${2:-data/bc_v2_gazebo}"
MAX_DURATION="${3:-120.0}"

echo "================================================"
echo "  BC 数据采集"
echo "  模式: $MODE"
echo "  输出: $OUT_DIR"
echo "  单集最长: ${MAX_DURATION}s"
echo "================================================"

# -------------------------------------------------------
# 1. 启动 Gazebo 仿真 (PX4 + 双机 + 相机桥接)
# -------------------------------------------------------
echo ""
echo "[1/5] 启动 Gazebo 仿真..."
cd ~/PX4-Autopilot
./start_simulation.sh &
SIM_PID=$!
cd - > /dev/null

echo "  仿真 PID: $SIM_PID, 等待 PX4 就绪 (30s)..."
for i in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -q "/px4_1/fmu/out/vehicle_status"; then
        echo "  ✓ PX4 已就绪"
        break
    fi
    sleep 1
done

# -------------------------------------------------------
# 2. 启动目标机仿真
# -------------------------------------------------------
echo ""
echo "[2/5] 启动目标机 (mode=$MODE)..."
ros2 run uav_target_sim uav_target_sim --ros-args -p motion_mode:="$MODE" &
TARGET_PID=$!
echo "  目标机 PID: $TARGET_PID"
sleep 3

# -------------------------------------------------------
# 3. 启动视觉检测
# -------------------------------------------------------
echo ""
echo "[3/5] 启动视觉检测..."
ros2 run uav_vision_dectect uav_vision_dectect &
DETECT_PID=$!
echo "  检测 PID: $DETECT_PID"
sleep 2

# -------------------------------------------------------
# 4. 启动 PNG 拦截 (GPS-based, 机头对准目标)
# -------------------------------------------------------
echo ""
echo "[4/5] 启动 PNG 拦截..."
ros2 run uav_png_intercept uav_png_intercept &
PNG_PID=$!
echo "  PNG PID: $PNG_PID"
sleep 3

# -------------------------------------------------------
# 5. 启动 BC 录制器
# -------------------------------------------------------
echo ""
echo "[5/5] 启动 BC 录制器..."
ros2 run uav_bc_recorder uav_bc_recorder \
    --ros-args -p out_dir:="$OUT_DIR" -p mode:="$MODE" -p max_duration:="$MAX_DURATION" &
RECORDER_PID=$!
echo "  录制器 PID: $RECORDER_PID"

echo ""
echo "================================================"
echo "  所有节点已启动"
echo "  SIM:      $SIM_PID"
echo "  TARGET:   $TARGET_PID"
echo "  DETECT:   $DETECT_PID"
echo "  PNG:      $PNG_PID"
echo "  RECORDER: $RECORDER_PID"
echo "================================================"
echo ""
echo "按 Ctrl+C 停止所有节点"

# -------------------------------------------------------
# 退出清理
# -------------------------------------------------------
cleanup() {
    echo ""
    echo "🛑 正在关闭所有节点..."
    kill "$RECORDER_PID" "$PNG_PID" "$DETECT_PID" "$TARGET_PID" 2>/dev/null || true
    sleep 1
    kill "$SIM_PID" 2>/dev/null || true
    sleep 1
    # 清理残留
    killall -9 px4 gz-sim gz-server gz-client MicroXRCEAgent parameter_bridge 2>/dev/null || true
    echo "✅ 清理完成"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 等待录制器退出
wait "$RECORDER_PID" 2>/dev/null
cleanup