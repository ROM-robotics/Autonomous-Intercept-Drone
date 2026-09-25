```
colcon build --packages-select uav_common_msg uav_vision_dectect --cmake-args -DONNXRUNTIME_ROOT=/home/mr_robot/.local/onnxruntime/1.16.3
```

```
export LD_LIBRARY_PATH=/home/mr_robot/.local/onnxruntime/1.16.3/lib:${LD_LIBRARY_PATH:-}
```