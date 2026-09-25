#!/usr/bin/env python3
"""BC 数据录制节点 —— 采集 Gazebo 仿真中 PNG 拦截过程的原始传感器数据

录制内容:
  - 视频: /camera/image → 每集一个 .mp4 (1920×1080)
  - CSV:  检测 bbox / 姿态 / 速度 / PNG 速度指令 / GPS 位置 / 距离

每集独立输出，训练预处理阶段再做特征构造和动作编码。

用法:
  ros2 run uav_bc_recorder uav_bc_recorder \
      --ros-args -p out_dir:=data/bc_v2_gazebo -p mode:=circle -p max_duration:=120.0
"""
import csv
import math
import os
import signal
import sys
import time

import cv2
import numpy as np
import rclpy
from px4_msgs.msg import (SensorGps, TrajectorySetpoint, VehicleLocalPosition,
                           VehicleOdometry)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from uav_common_msg.msg import RectMsg

# ---------------------------------------------------------------------------
#  Flat-earth GPS → NED
# ---------------------------------------------------------------------------
_M_PER_DEG_LAT = 111320.0


def gps_to_ned(lat, lon, alt, origin_lat, origin_lon, origin_alt):
    dn = (lat - origin_lat) * _M_PER_DEG_LAT
    de = (lon - origin_lon) * _M_PER_DEG_LAT * math.cos(math.radians(origin_lat))
    dd = -(alt - origin_alt)
    return dn, de, dd


# ======================================================================
#  ROS2 Image → numpy (不依赖 cv_bridge，避免 numpy 版本冲突)
# ======================================================================
_ENCODING_TO_CHANNELS = {
    "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4,
    "mono8": 1, "8uc1": 1, "8uc3": 3, "8uc4": 4,
}


def imgmsg_to_numpy(msg: Image) -> np.ndarray:
    """将 sensor_msgs/Image 解码为 numpy 数组 (H, W, C)"""
    c = _ENCODING_TO_CHANNELS.get(msg.encoding, 3)
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, c)


# ======================================================================
#  Node
# ======================================================================
class BCRecorder(Node):
    def __init__(self):
        super().__init__("uav_bc_recorder")

        # ---- 参数 ----
        self.declare_parameter("out_dir", "data/bc_v2_gazebo")
        self.declare_parameter("mode", "circle")
        self.declare_parameter("max_duration", 120.0)

        self._out_dir = self.get_parameter("out_dir").value
        self._mode = self.get_parameter("mode").value
        self._max_duration = self.get_parameter("max_duration").value

        # ---- QoS (BEST_EFFORT, 与 PX4 一致) ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---- 话题缓存 (latest-value) ----
        self._image = None        # np.ndarray (H, W, 3) BGR
        self._det = None          # (x, y, w, h) or None
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._local_z = 0.0
        self._cmd_vx = float("nan")
        self._cmd_vy = float("nan")
        self._cmd_vz = float("nan")
        self._cmd_yawrate = float("nan")
        self._self_n = 0.0
        self._self_e = 0.0
        self._self_d = 0.0
        self._tgt_n = 0.0
        self._tgt_e = 0.0
        self._tgt_d = 0.0
        self._gps_origin = None
        self._tgt_gps_ok = False
        self._self_gps_ok = False

        # ---- 订阅者 ----
        self._image_sub = self.create_subscription(
            Image, "/camera/image", self._image_cb, 10)

        self._detect_sub = self.create_subscription(
            RectMsg, "/camera_detect_result", self._detect_cb, 10)

        self._odom_sub = self.create_subscription(
            VehicleOdometry, "/px4_1/fmu/out/vehicle_odometry",
            self._odom_cb, qos)

        self._lp_sub = self.create_subscription(
            VehicleLocalPosition, "/px4_1/fmu/out/vehicle_local_position",
            self._lp_cb, qos)

        self._traj_sub = self.create_subscription(
            TrajectorySetpoint, "/px4_1/fmu/in/trajectory_setpoint",
            self._traj_cb, 10)

        self._self_gps_sub = self.create_subscription(
            SensorGps, "/px4_1/fmu/out/vehicle_gps_position",
            self._self_gps_cb, qos)

        self._tgt_gps_sub = self.create_subscription(
            SensorGps, "/px4_2/fmu/out/vehicle_gps_position",
            self._tgt_gps_cb, qos)

        # ---- 状态机 ----
        self._state = "IDLE"
        self._episode = 0
        self._frame_idx = 0
        self._ep_start_time = 0.0
        self._lost_count = 0
        self._max_lost = 90

        # ---- 写入器 (延迟初始化) ----
        self._video_writer = None
        self._csv_file = None
        self._csv_writer = None

        # ---- 汇总 CSV ----
        self._summary_path = os.path.join(self._out_dir, "summary.csv")

        os.makedirs(self._out_dir, exist_ok=True)

        self.get_logger().info(
            f"uav_bc_recorder 已启动 | out={self._out_dir} mode={self._mode} "
            f"max_duration={self._max_duration}s"
        )
        self.get_logger().info(
            "等待 PNG 进入 INTERCEPT 阶段..."
        )

    # ==================================================================
    #  回调: 缓存最新值
    # ==================================================================

    def _image_cb(self, msg: Image):
        try:
            self._image = imgmsg_to_numpy(msg)
        except Exception:
            return
        self._maybe_record()

    def _detect_cb(self, msg: RectMsg):
        if msg.width == -1:
            self._det = None
        else:
            self._det = (msg.x, msg.y, msg.width, msg.height)

    def _odom_cb(self, msg: VehicleOdometry):
        q = msg.q
        self._roll = math.atan2(
            2 * (q[0] * q[1] + q[2] * q[3]),
            1 - 2 * (q[1] * q[1] + q[2] * q[2]))
        s2 = max(-1.0, min(1.0, 2 * (q[0] * q[2] - q[3] * q[1])))
        self._pitch = math.asin(s2)
        self._yaw = math.atan2(
            2 * (q[0] * q[3] + q[1] * q[2]),
            1 - 2 * (q[2] * q[2] + q[3] * q[3]))

    def _lp_cb(self, msg: VehicleLocalPosition):
        self._vx = msg.vx
        self._vy = msg.vy
        self._vz = msg.vz
        self._local_z = msg.z

    def _traj_cb(self, msg: TrajectorySetpoint):
        self._cmd_vx = msg.velocity[0]
        self._cmd_vy = msg.velocity[1]
        self._cmd_vz = msg.velocity[2]
        self._cmd_yawrate = msg.yawspeed

    def _self_gps_cb(self, msg: SensorGps):
        if self._gps_origin is None:
            self._gps_origin = (msg.latitude_deg, msg.longitude_deg,
                                msg.altitude_msl_m)
        self._self_n, self._self_e, self._self_d = gps_to_ned(
            msg.latitude_deg, msg.longitude_deg, msg.altitude_msl_m,
            *self._gps_origin)
        self._self_gps_ok = True

    def _tgt_gps_cb(self, msg: SensorGps):
        if self._gps_origin is None:
            return
        self._tgt_n, self._tgt_e, self._tgt_d = gps_to_ned(
            msg.latitude_deg, msg.longitude_deg, msg.altitude_msl_m,
            *self._gps_origin)
        self._tgt_gps_ok = True

    # ==================================================================
    #  核心: 录制逻辑
    # ==================================================================

    def _cmd_is_active(self) -> bool:
        for v in (self._cmd_vx, self._cmd_vy, self._cmd_vz):
            if math.isnan(v) or abs(v) < 1e-6:
                return False
        return True

    def _maybe_record(self):
        if self._image is None:
            return

        if self._state == "IDLE":
            if self._det is not None and self._cmd_is_active():
                self._start_episode()
                self._write_frame()
        elif self._state == "RECORDING":
            self._write_frame()
            self._check_episode_end()

    def _start_episode(self):
        self._episode += 1
        self._frame_idx = 0
        self._ep_start_time = time.time()
        self._lost_count = 0

        h, w = self._image.shape[:2]
        stem = f"ep_{self._episode:03d}_{self._mode}"

        video_path = os.path.join(self._out_dir, stem + ".mp4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self._video_writer = cv2.VideoWriter(video_path, fourcc, 30.0, (w, h))
        if not self._video_writer.isOpened():
            self.get_logger().error(f"无法创建视频文件: {video_path}")
            self._state = "IDLE"
            return

        csv_path = os.path.join(self._out_dir, stem + ".csv")
        self._csv_file = open(csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "frame_idx",
            "det_x", "det_y", "det_w", "det_h",
            "roll", "pitch", "yaw",
            "vx", "vy", "vz", "local_z",
            "cmd_vx", "cmd_vy", "cmd_vz", "cmd_yawrate",
            "self_n", "self_e", "self_d",
            "tgt_n", "tgt_e", "tgt_d",
            "dist",
        ])

        self._state = "RECORDING"
        self.get_logger().info(
            f"[EP {self._episode}] 开始录制 | mode={self._mode} "
            f"video={stem}.mp4")

    def _write_frame(self):
        if self._video_writer is None or self._csv_writer is None:
            return

        self._video_writer.write(self._image)

        if self._self_gps_ok and self._tgt_gps_ok:
            dist = math.sqrt(
                (self._self_n - self._tgt_n) ** 2
                + (self._self_e - self._tgt_e) ** 2
                + (self._self_d - self._tgt_d) ** 2)
        else:
            dist = -1.0

        det = self._det
        self._csv_writer.writerow([
            self._frame_idx,
            det[0] if det else -1,
            det[1] if det else -1,
            det[2] if det else -1,
            det[3] if det else -1,
            round(self._roll, 6),
            round(self._pitch, 6),
            round(self._yaw, 6),
            round(self._vx, 4),
            round(self._vy, 4),
            round(self._vz, 4),
            round(self._local_z, 4),
            round(self._cmd_vx, 4) if not math.isnan(self._cmd_vx) else "",
            round(self._cmd_vy, 4) if not math.isnan(self._cmd_vy) else "",
            round(self._cmd_vz, 4) if not math.isnan(self._cmd_vz) else "",
            round(self._cmd_yawrate, 6) if not math.isnan(self._cmd_yawrate) else "",
            round(self._self_n, 4),
            round(self._self_e, 4),
            round(self._self_d, 4),
            round(self._tgt_n, 4),
            round(self._tgt_e, 4),
            round(self._tgt_d, 4),
            round(dist, 4),
        ])

        self._frame_idx += 1

        if self._det is None:
            self._lost_count += 1
        else:
            self._lost_count = 0

    def _check_episode_end(self):
        elapsed = time.time() - self._ep_start_time

        if self._self_gps_ok and self._tgt_gps_ok:
            dist = math.sqrt(
                (self._self_n - self._tgt_n) ** 2
                + (self._self_e - self._tgt_e) ** 2
                + (self._self_d - self._tgt_d) ** 2)
            if dist < 0.8:
                self._end_episode("hit")
                return

        if self._lost_count >= self._max_lost:
            self._end_episode("lost")
            return

        if elapsed > self._max_duration:
            self._end_episode("timeout")
            return

    def _end_episode(self, outcome: str):
        elapsed = time.time() - self._ep_start_time

        if self._video_writer:
            self._video_writer.release()
            self._video_writer = None

        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None

        self._write_summary(self._episode, self._mode, outcome,
                            self._frame_idx, elapsed)

        self.get_logger().info(
            f"[EP {self._episode}] 结束 | outcome={outcome} "
            f"frames={self._frame_idx} duration={elapsed:.1f}s"
        )

        self._state = "IDLE"
        self.get_logger().info(
            "等待下一集开始 (切换目标模式后 PNG 重新检测到目标即自动开始)..."
        )

    def _write_summary(self, episode, mode, outcome, num_frames, duration):
        existed = os.path.exists(self._summary_path)
        with open(self._summary_path, "a", newline="") as f:
            w = csv.writer(f)
            if not existed:
                w.writerow(["episode", "mode", "outcome", "num_frames",
                            "duration_s"])
            w.writerow([episode, mode, outcome, num_frames,
                        round(duration, 1)])

    # ==================================================================
    def destroy_node(self):
        if self._video_writer:
            self._video_writer.release()
        if self._csv_file:
            self._csv_file.close()
        super().destroy_node()


# ======================================================================
def main():
    rclpy.init()
    node = BCRecorder()

    def _sigint(signum, frame):
        node.get_logger().info("收到 SIGINT，正在关闭...")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()