# How to Train LightTrack — အစအဆုံး လမ်းညွှန်

> ဤလမ်းညွှန်သည် **LightTrack (Single Object Tracker)** မော်ဒယ်အား ကိုယ်ပိုင် Drone Dataset သို့မဟုတ် Open Dataset များဖြင့် မည်သို့ Train ရမည်၊ ထို့နောက် `lighttrack_init.onnx` နှင့် `lighttrack_update.onnx` အဖြစ် မည်သို့ Export ထုတ်ပြီး ဤ Project တွင် ထည့်သွင်းအသုံးပြုရမည်ကို အဆင့်ဆင့် ဖော်ပြထားပါသည်။

---

## ၁။ ခြုံငုံသုံးသပ်ချက် (Architecture Overview)

LightTrack သည် **Siamese Tracking Framework** အပေါ် အခြေခံထားပြီး မော်ဒယ် (၂) ခုအဖြစ် ခွဲထုတ်ထားပါသည် -

| မော်ဒယ်ဖိုင် | Input | Output | လုပ်ဆောင်ချက် |
|---|---|---|---|
| **`lighttrack_init.onnx`** | `[1, 3, 127, 127]` (Target Crop) | `[1, 96, 8, 8]` ($z_f$ Feature) | ပစ်မှတ်၏ အသွင်အပြင် Template ကို ၁ ကြိမ်သာ Feature ထုတ်ယူသည်။ |
| **`lighttrack_update.onnx`** | `[1, 96, 8, 8]` ($z_f$) + `[1, 3, 288, 288]` (Search Crop) | Cls `[1, 1, 18, 18]` + Reg `[1, 4, 18, 18]` | Frame တိုင်းတွင် ပစ်မှတ် မည်သည့်နေရာသို့ ရွေ့သွားသည်ကို High FPS ဖြင့် ရှာဖွေသည်။ |

---

## ၂။ Environment ပြင်ဆင်ခြင်း (Prerequisites)

```bash
# 1. Virtual Environment ဖန်တီးပါ
conda create -n lighttrack python=3.8 -y
conda activate lighttrack

# 2. PyTorch (CUDA ထောက်ပံ့သော ဗားရှင်း) သွင်းပါ
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# 3. လိုအပ်သော Libraries များ သွင်းပါ
pip install onnx onnxruntime opencv-python numpy yacs timm tqdm
```

---

## ၃။ Dataset ပြင်ဆင်ခြင်း (Dataset Preparation)

Visual Tracking အတွက် အောက်ပါ Dataset များကို အသုံးပြုနိုင်ပါသည်-

1. **Standard Tracking Datasets**:
   * **LaSOT** (Large-scale Single Object Tracking)
   * **GOT-10k** (Generic Object Tracking)
   * **TrackingNet**
2. **Drone သီးသန့် Dataset များ**:
   * **UAV123** (ဒရုန်းဖြင့် ရိုက်ကူးထားသော ဗီဒီယို Tracking Dataset)
   * **Custom Simulation Dataset**: Gazebo ထဲတွင် Interceptor Drone ၏ `/camera/image` မှ ဗီဒီယိုများကို Record လုပ်ပြီး CVAT သို့မဟုတ် LabelImg ဖြင့် Bounding Box Annotate ပြုလုပ်ထားသော ဒေတာများ။

### Data Structure:
```text
datasets/
├── drone_tracking/
│   ├── video_001/
│   │   ├── 000001.jpg
│   │   ├── 000002.jpg
│   │   └── groundtruth.txt   # frame တိုင်းအတွက် [x, y, w, h]
│   └── video_002/
│       ├── 000001.jpg
│       └── ...
```

---

## ၄။ Model လေ့ကျင့်သင်ကြားခြင်း (Training Process)

မူရင်း LightTrack Repo ကို အသုံးပြု၍ လေ့ကျင့်နိုင်ပါသည်:

```bash
git clone https://github.com/researchmm/LightTrack.git
cd LightTrack
```

### က။ Data Sampling & Augmentation
Training လုပ်ဆောင်စဉ် စနစ်သည် ဗီဒီယိုတစ်ခုထဲမှ Frame (၂) ခုကို တွဲဖက်ရွေးချယ်သည်:
* **Exemplar/Template Frame ($z$)**: ပစ်မှတ်ဗဟိုပြု `127×127` သို့ Crop လုပ်သည်။
* **Search Frame ($x$)**: အနည်းငယ်ကွာသော Frame မှ ပစ်မှတ် ပတ်လည် `288×288` သို့ Crop လုပ်သည်။
* **Augmentations**: Scale jitter, translation, color jitter, blur များကို ထည့်သွင်းပေးသည်။

### ခ။ Loss Functions (အမှားတွက်ချက်မှု)
Total Loss သည် အပိုင်း (၂) ပိုင်း ပါဝင်သည်-
$$\mathcal{L}_{total} = \mathcal{L}_{cls} + \lambda \mathcal{L}_{reg}$$

1. **Classification Loss ($\mathcal{L}_{cls}$)**:
   * ပစ်မှတ်၏ ဗဟိုနေရာကို Background နှင့် ကွဲပြားစွာ ခွဲခြားရန် **Binary Cross-Entropy (BCE)** သို့မဟုတ် **Focal Loss** ကို သုံးသည်။
2. **Bounding Box Regression Loss ($\mathcal{L}_{reg}$)**:
   * တည်နေရာနှင့် အရွယ်အစား အတိအကျရရှိစေရန် **GIoU (Generalized Intersection over Union) Loss** ကို သုံးသည်။

### ဂ။ Training Command နမူနာ
```bash
python tracking/train.py \
    --cfg experiments/lighttrack/lighttrack_mobile.yaml \
    --batch_size 32 \
    --epochs 50 \
    --lr 0.005 \
    --save_dir checkpoints/
```

---

## ၅။ ONNX သို့ Model (၂) ခု ခွဲထုတ်ခြင်း (Exporting to ONNX)

Train ပြီးစီးပါက ရရှိလာသော PyTorch Checkpoint (`.pth`) ဖိုင်အား ဤ Project ၏ C++ Code နှင့် ကိုက်ညီအောင် **Init** နှင့် **Update** ဟူ၍ ဖိုင် (၂) ခု ခွဲထုတ်ရပါမည်။

အောက်ပါ Python Script ဖြင့် Export ပြုလုပ်နိုင်ပါသည်:

```python
import torch
import torch.nn as nn
from models.lighttrack import build_lighttrack  # LightTrack architecture

# 1. Checkpoint Load လုပ်ခြင်း
checkpoint = torch.load("checkpoints/lighttrack_best.pth", map_location="cpu")
model = build_lighttrack()
model.load_state_dict(checkpoint["model"])
model.eval()

# ============================================================
# A. Export lighttrack_init.onnx
# ============================================================
class LightTrackInit(nn.Module):
    def __init__(self, model):
        super().__init__()
        self.backbone = model.backbone

    def forward(self, z):
        # z: [1, 3, 127, 127] -> zf: [1, 96, 8, 8]
        zf = self.backbone(z)
        return zf

init_model = LightTrackInit(model)
dummy_z = torch.randn(1, 3, 127, 127)

torch.onnx.export(
    init_model,
    dummy_z,
    "lighttrack_init.onnx",
    input_names=["input1"],
    output_names=["output.1"],
    opset_version=11
)
print("✅ Exported lighttrack_init.onnx successfully!")

# ============================================================
# B. Export lighttrack_update.onnx
# ============================================================
class LightTrackUpdate(nn.Module):
    def __init__(self, model):
        super().__init__()
        self.backbone = model.backbone
        self.neck = model.neck
        self.head = model.head

    def forward(self, zf, x):
        # zf: [1, 96, 8, 8], x: [1, 3, 288, 288]
        xf = self.backbone(x)
        features = self.neck(zf, xf)
        cls_score, bbox_reg = self.head(features)
        # cls_score: [1, 1, 18, 18], bbox_reg: [1, 4, 18, 18]
        return cls_score, bbox_reg

update_model = LightTrackUpdate(model)
dummy_zf = torch.randn(1, 96, 8, 8)
dummy_x = torch.randn(1, 3, 288, 288)

torch.onnx.export(
    update_model,
    (dummy_zf, dummy_x),
    "lighttrack_update.onnx",
    input_names=["input1", "input2"],
    output_names=["output.1", "output.2"],
    opset_version=11
)
print("✅ Exported lighttrack_update.onnx successfully!")
```

---

## ၆။ ဤ Project တွင် Model အသစ် ထည့်သွင်းအသုံးပြုခြင်း

Export လုပ်ပြီး ရရှိလာသော မော်ဒယ်ဖိုင် (၂) ခုကို အောက်ပါအတိုင်း အစားထိုးနိုင်ပါသည်-

1. **ဖိုင်များ ကူးယူပါ**:
   ```bash
   cp lighttrack_init.onnx /home/hmue_gyi/Desktop/Git/Autonomous_Intercept_Drone/uav_vision_dectect/model/light_track/
   cp lighttrack_update.onnx /home/hmue_gyi/Desktop/Git/Autonomous_Intercept_Drone/uav_vision_dectect/model/light_track/
   ```

2. **C++ Code ထဲရှိ Model Path ကို စစ်ဆေးပါ**:
   [`uav_vision_dectect/src/uav_topic_subscrib.cpp`](file:///home/hmue_gyi/Desktop/Git/Autonomous_Intercept_Drone/uav_vision_dectect/src/uav_topic_subscrib.cpp) ၏ Line 48–51 တွင် သင့်လက်ရှိ Folder လမ်းကြောင်း မှန်ကန်ကြောင်း သေချာအောင် စစ်ဆေးပါ:
   ```cpp
   std::string init_model = "/home/hmue_gyi/Desktop/Git/Autonomous_Intercept_Drone/uav_vision_dectect/model/light_track/lighttrack_init";
   std::string update_model = "/home/hmue_gyi/Desktop/Git/Autonomous_Intercept_Drone/uav_vision_dectect/model/light_track/lighttrack_update";
   siam_tracker = new LightTrack(init_model.c_str(), update_model.c_str());
   ```

3. **Project ကို ပြန်လည် Build လုပ်ပါ**:
   ```bash
   cd ~/Desktop/Git/Autonomous_Intercept_Drone
   colcon build --packages-select uav_vision_dectect
   source install/setup.bash
   ```

---

## ၇။ အကျဉ်းချုပ်

* **LightTrack** သည် Siamese Architecture ကို သုံးထားပြီး **Template (`127×127`)** နှင့် **Search Area (`288×288`)** ကို Cross-correlation ပြုလုပ်ကာ ပစ်မှတ်ကို ခြေရာခံသည်။
* Train ရာတွင် **Classification Loss (BCE/Focal)** နှင့် **Regression Loss (GIoU)** ကို ပေါင်းစပ်အသုံးပြုသည်။
* C++ တွင် အလွန်လျင်မြန်စွာ run နိုင်ရန် `lighttrack_init.onnx` (Template Feature ရယူရန်) နှင့် `lighttrack_update.onnx` (Frame အလိုက် နေရာအသစ်တွက်ရန်) ဟူ၍ ခွဲထုတ်အသုံးပြုထားပါသည်။
