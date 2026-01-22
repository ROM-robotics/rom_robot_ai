# How to Run (TurtleBot3 VLM Navigation)

ဤပရောဂျက်ကို run ရန် အောက်ပါအဆင့်များကို လုပ်ဆောင်ပါ။

## 1. Prerequisites (လိုအပ်ချက်များ)
- **ROS 2 Humble** installed.
- **TurtleBot3** packages installed (`sudo apt install ros-humble-turtlebot3*`).
- **Nav2** packages installed (`sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`).
- **OpenAI API Key** (GPT-4o အသုံးပြုရန်).
- Python libraries: `openai`, `numpy`, `opencv-python`.

## 2. Environment Setup (ပတ်ဝန်းကျင်သတ်မှတ်ခြင်း)
Terminal တွင် အောက်ပါ environment variable များကို သတ်မှတ်ပေးရပါမည်။

```bash
# TurtleBot3 Model ကိုသတ်မှတ်ပါ (burger, waffle, waffle_pi)
export TURTLEBOT3_MODEL=waffle

# OpenAI API Key ကိုထည့်ပါ
export OPENAI_API_KEY='your-api-key-here'

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

## 3. Map Preparation (မြေပုံပြင်ဆင်ခြင်း)
Script များသည် `maps/` folder အောက်ရှိ map file များအား ယူသုံးထားသောကြောင့် အောက်ပါဖိုင်များ ရှိနေကြောင်း သေချာပါစေ။
- `maps/apartment.yaml`
- `maps/apartment.pgm` (သို့မဟုတ် `.png`)
- `src/nav2-vlm/input.png` (AI အတွက်သုံးမည့် Map image)

> **မှတ်ချက်**: အကယ်၍ မြေပုံမရှိသေးပါက SLAM အသုံးပြု၍ အရင်ထုတ်ယူရပါမည်။

## 4. Simulation & Navigation Stack (စနစ်စတင်ခြင်း)
 `run_project.bash` script ကို အသုံးပြု၍ လိုအပ်သော nodes များကို တပြိုင်နက် run နိုင်ပါသည်။

```bash
cd src/nav2-vlm
chmod +x run_project.bash
./run_project.bash
```

သို့မဟုတ် Terminal ၃ ခုခွဲ၍ တစ်ခုချင်းစီ run နိုင်သည် (Manual run):

**Terminal 1 (Simulation):**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_apartment_1.launch.py
```

**Terminal 2 (Localization):**
```bash
ros2 launch nav2_bringup localization_launch.py map:=maps/apartment.yaml
```

**Terminal 3 (Navigation):**
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/apartment.yaml
```

## 5. Running AI Planner (AI စနစ်မောင်းနှင်ခြင်း)
Simulation နှင့် Nav2 အဆင်သင့်ဖြစ်ပါက (RViz တွင် Robot ကိုမြင်ရပြီး Localize ဖြစ်ပါက) နောက်ထပ် Terminal တစ်ခုတွင် AI script ကို run ပါ။

```bash
source /opt/ros/humble/setup.bash
cd src/nav2-vlm
python3 nav2_test.py
```

Code run လိုက်သောအခါ
1. Robot သည် Initial Pose ကို အလိုအလျောက် သတ်မှတ်ပါလိမ့်မည်။
2. `Where do you want to go? :` ဟု မေးပါက သွားလိုသည့်နေရာ (ဥပမာ - "Dining Room") ကို ရိုက်ထည့်ပါ။
3. AI က မြေပုံ (`input.png`) ကိုကြည့်ပြီး လမ်းကြောင်းတွက်ချက်ပေးလိမ့်မည်။
4. Robot သည် သတ်မှတ်ထားသော နေရာသို့ စတင်ရွေ့လျားပါလိမ့်မည်။