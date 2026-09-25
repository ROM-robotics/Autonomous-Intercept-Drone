# Autonomous Intercept Drone — Build & Run TODO

ဒီဖိုင်ကို အပေါ်ကနေ အောက်အထိ အစဉ်လိုက်လုပ်ပါ။ Command များကို သတ်မှတ်ထားသော directory မှာပဲ run ပါ။

## ၁။ လိုအပ်ချက်များ စစ်ဆေးပါ

- Ubuntu 22.04
- ROS 2 Humble
- PX4-Autopilot v1.16
- Gazebo Harmonic
- Micro-XRCE-DDS-Agent
- OpenCV, Eigen3, GeographicLib
- ONNX Runtime C++

ROS 2 ရှိ/မရှိ စစ်ပါ-

```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
```

`humble` ဟု ပြရပါမယ်။ မပြပါက ROS 2 Humble ကို အရင် install လုပ်ပါ။

လိုအပ်သော Ubuntu/ROS packages များ install လုပ်ပါ-

```bash
sudo apt update
sudo apt install -y \
  apache2-utils \
  libgeographic-dev \
  ros-humble-geographic-msgs \
  ros-humble-cv-bridge \
  libopencv-dev \
  libeigen3-dev
```

## ၂။ Workspace လမ်းကြောင်းကို မှန်အောင်သုံးပါ

ဒီ project ရဲ့ workspace root က-

```text
/home/username/dev_ws
```

```bash
cd /home/username/dev_ws
```

`colcon build` ကို `src` directory ထဲက မ run ပါနှင့်။ Source packages များသည်-

```text
/home/username/dev_ws/src/Autonomous_Intercept_Drone/
```

## ၃။ ONNX Runtime C++ ကို install လုပ်ပါ

`uav_rl_guidance` နှင့် `uav_vision_dectect` build အတွက် ONNX Runtime C++ လိုအပ်ပါသည်။

```bash
cd /tmp
wget https://github.com/microsoft/onnxruntime/releases/download/v1.23.0/onnxruntime-linux-x64-1.23.0.tgz
tar -xzf onnxruntime-linux-x64-1.23.0.tgz
cd onnxruntime-linux-x64-1.23.0

sudo mkdir -p /usr/local/include /usr/local/lib
sudo cp -r include/* /usr/local/include/
sudo cp include/onnxruntime/core/session/*.h /usr/local/include/
sudo cp lib/libonnxruntime.so* /usr/local/lib/
sudo ldconfig
```

Install အောင်မြင်/မအောင်မြင် စစ်ပါ-

```bash
test -f /usr/local/include/onnxruntime_cxx_api.h && echo "ONNX header OK"
find /usr/local/lib -name 'libonnxruntime.so*'
```

`ONNX header OK` နှင့် `libonnxruntime.so` ပါသော output ပြရပါမယ်။ CPU ONNX Runtime သုံးရန် core library တစ်ခုတည်း လုံလောက်ပါသည်။

## ၄။ rosdep ဖြင့် dependency များ စစ်ပါ

```bash
cd /home/username/dev_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## ၅။ PX4-Autopilot ကို တစ်ကြိမ်ပြင်ဆင်ပါ

PX4 မရှိသေးပါက-

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Custom world နှင့် startup script ကို PX4 ထဲသို့ ကူးပါ-

```bash
cp /home/username/dev_ws/src/Autonomous_Intercept_Drone/assets/gazebo_world/grass_world.sdf \
  ~/PX4-Autopilot/Tools/simulation/gz/worlds/

cp /home/username/dev_ws/src/Autonomous_Intercept_Drone/run_swarm.sh \
  ~/PX4-Autopilot/
chmod +x ~/PX4-Autopilot/run_swarm.sh
```

## ၆။ Message packages များကို အရင် build လုပ်ပါ

```bash
cd /home/username/dev_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select px4_msgs uav_common_msg
```

အဆုံးမှာ `Finished <<< px4_msgs` နှင့် `Finished <<< uav_common_msg` ပြရပါမယ်။

## ၇။ Workspace တစ်ခုလုံး build လုပ်ပါ

```bash
cd /home/username/dev_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /home/username/dev_ws/install/setup.bash
```

Terminal အသစ်ဖွင့်တိုင်း ROS နှင့် workspace ကို source လုပ်ပါ-

```bash
source /opt/ros/humble/setup.bash
source /home/username/dev_ws/install/setup.bash
```

## ၈။ Build error ဖြေရှင်းနည်း

### `Could not find ONNXRUNTIME_LIB`

အခန်း ၃ ကို ပြန်လုပ်ပြီး အောက်ပါအတိုင်း စစ်ပါ-

```bash
find /usr/local/lib -name 'libonnxruntime.so*'
test -f /usr/local/include/onnxruntime_cxx_api.h && echo OK
```

### `uav_common_msgConfig.cmake` မတွေ့ခြင်း

Message package မပြီးသေးခြင်း သို့မဟုတ် build order မမှန်ခြင်း ဖြစ်ပါသည်။

```bash
cd /home/username/dev_ws
colcon build --symlink-install --packages-select px4_msgs uav_common_msg
colcon build --symlink-install
```

`uav_vision_dectect` ၏ dependency name သည် `uav_common_msg` ဖြစ်ရပါမယ်။ `uav_common_msgs` မဟုတ်ပါ။

### `existing path cannot be removed: Is a directory`

အရင် build artifact နဲ့ `--symlink-install` တိုက်နေခြင်း ဖြစ်ပါသည်။ `<package_name>` နေရာတွင် error ပြသော package အမည်ထည့်ပါ-

```bash
cd /home/username/dev_ws
mv build/<package_name> build/<package_name>.backup
colcon build --symlink-install --packages-select <package_name>
```

ဥပမာ-

```bash
mv build/uav_common_msg build/uav_common_msg.backup
colcon build --symlink-install --packages-select uav_common_msg
```

### `ONNXRUNTIME_CUDA_LIB` သို့မဟုတ် `ONNXRUNTIME_SHARED_LIB` `NOTFOUND`

CPU mode သုံးလျှင် CUDA/shared provider မလိုပါ။ Core `libonnxruntime.so` ရှိပါက build ဆက်လုပ်နိုင်ပါသည်။ GPU inference လိုအပ်မှသာ CUDA provider ကို install လုပ်ပါ။

## ၉။ Simulation ကို စတင်ပါ

### Terminal 1 — PX4, Gazebo နှင့် DDS Agent

```bash
cd ~/PX4-Autopilot
./run_swarm.sh
```

Gazebo နှင့် drone များ load ဖြစ်ရန် ၁၅ စက္ကန့်ခန့် စောင့်ပါ။

### Terminal 2 — Target simulator

```bash
source /opt/ros/humble/setup.bash
source /home/username/dev_ws/install/setup.bash
ros2 run uav_target_sim uav_target_sim
```

### Terminal 3 — Vision detection

```bash
source /opt/ros/humble/setup.bash
source /home/username/dev_ws/install/setup.bash
ros2 run uav_vision_dectect uav_vision_dectect
```

### Terminal 4 — Guidance

RL guidance သုံးမည်ဆိုပါက-

```bash
source /opt/ros/humble/setup.bash
source /home/username/dev_ws/install/setup.bash
ros2 launch uav_rl_guidance rl_guidance.launch.py
```

PNG guidance သုံးမည်ဆိုပါက-

```bash
source /opt/ros/humble/setup.bash
source /home/username/dev_ws/install/setup.bash
ros2 run uav_vision_png uav_vision_png
```

## ၁၀။ စနစ်အလုပ်လုပ်/မလုပ် စစ်ပါ

```bash
ros2 topic list | grep -E "camera|detect|px4_1"
ros2 topic hz /camera/image
ros2 topic echo /camera_detect_result
ros2 topic echo /px4_1/fmu/in/trajectory_setpoint
```

## ၁၁။ စနစ်ပိတ်ပါ

Terminal တစ်ခုချင်းစီတွင် `Ctrl+C` နှိပ်ပါ။ Process များကျန်နေပါက-

```bash
killall -9 px4 gz-sim gz-server gz-client MicroXRCEAgent parameter_bridge 2>/dev/null || true
```
