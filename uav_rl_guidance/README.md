# uav_rl_guidance

基于**强化学习 (GRU 策略)** 的视觉制导拦截节点，是 `uav_vision_png` 的**即插即用替代方案**。

## 核心思路

传统方案使用比例导引（PNG）算法计算拦截轨迹。本方案使用**离线训练 → 在线推理**的强化学习策略取代 PNG：

```
/camera_detect_result  →  15维观测特征  →  GRU策略 (ONNX Runtime)  →  速度指令
                                                        ↓ (异常时)
                                                 内置 PNG 回退控制器
```

- **策略**：GRU 循环神经网络（BC + PPO 训练），输入 bbox 视觉特征 + 自身状态，输出 NED 速度指令
- **推理**：ONNX Runtime，零 PyTorch 依赖，纯 C++ 实现
- **安全兜底**：策略异常时自动回退到内置 PNG 控制器（watchdog 机制）
- **A/B 对比**：`fallback_png:=true` 可切换为纯 PNG 基线模式

## 与 uav_vision_png 的关系

| | uav_vision_png | uav_rl_guidance |
|---|---|---|
| 制导算法 | 比例导引 (PNG) | GRU 强化学习策略 |
| 话题接口 | 完全相同 | ← 兼容 |
| 状态机 | TAKE_OFF → SEARCHING → INTERCEPT → ... | ← 完全一致 |
| CSV 格式 | 完全相同列 | ← 兼容 |
| 实现语言 | C++ | C++ |
| 模型依赖 | 无 | ONNX Runtime (~15MB) |

## 文件结构

```
uav_rl_guidance/
├── CMakeLists.txt                  # ament_cmake 构建
├── package.xml
├── README.md
├── config/
│   └── params.yaml                 # 参数配置（模型路径、PNG 增益、丢失阈值等）
├── launch/
│   └── rl_guidance.launch.py       # 启动文件
├── models/
│   ├── policy.onnx                 # ONNX 推理模型（运行时加载）
│   ├── policy.pt                   # TorchScript 模型（用于重新导出 ONNX）
│   └── policy_meta.json            # 模型元信息（动作解码参数等）
├── include/uav_rl_guidance/
│   └── rl_guidance_node.hpp        # C++ 类声明
└── src/
    ├── rl_guidance_node.cpp        # C++ 完整实现
    └── export_onnx.py              # ONNX 导出脚本（模型更新时使用）
```

## 依赖

| 依赖 | 用途 |
|---|---|
| ONNX Runtime | GRU 策略推理 |
| Eigen3 | 几何运算（姿态旋转、LOS 计算） |
| GeographicLib | GPS→NED 坐标变换（统计用） |
| rclcpp / px4_msgs / uav_common_msg | ROS 2 通信 |

## 启动方式

```bash
# 标准启动（加载 ONNX 策略）
ros2 launch uav_rl_guidance rl_guidance.launch.py

# 纯 PNG 基线模式（A/B 对比，不加载策略）
ros2 launch uav_rl_guidance rl_guidance.launch.py fallback_png:=true

# 台架调试（跳过起飞，直接进入 SEARCHING）
ros2 launch uav_rl_guidance rl_guidance.launch.py bench_test:=true
```

## 主要参数

参见 `config/params.yaml`，关键参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `model_path` | `models/policy.onnx` | ONNX 模型路径 |
| `fallback_png` | `false` | 纯 PNG 基线模式 |
| `bench_test` | `false` | 跳过起飞（台架调试） |
| `standby_altitude` | `-6.0` | 待机高度（NED，负值向上） |
| `speed_cmd` | `5.0` | 指令速度 (m/s) |
| `kv` / `kz` | `4.0` | PNG 俯仰/方位增益 |
| `coast_thresh` | `30` | 惯性续飞帧数 |
| `lost_thresh` | `90` | 目标丢失切换 TRACK_LOST 帧数 |
| `hit_radius` | `0.8` | 命中判定半径 (m) |

## 模型更新

当 `guidance_rl` 训练出新策略后，重新导出 ONNX：

```bash
cd uav_rl_guidance
python3 src/export_onnx.py
```

该脚本读取 `models/policy.pt`，生成 `models/policy.onnx`。

## Watchdog 安全机制

RL 策略输出异常时自动回退：

| 触发条件 | 处理 |
|---|---|
| 策略输出 NaN/Inf | 回退 PNG，连续 20 帧（1s），重置 GRU 隐藏状态 |
| 速度指令 > 1.5×speed_cmd | 同上 |
| 推理异常（ONNX 报错等） | 同上 |

回退期间 PNG 控制器接管，策略隐藏状态保留。20 帧后自动恢复策略推理。
