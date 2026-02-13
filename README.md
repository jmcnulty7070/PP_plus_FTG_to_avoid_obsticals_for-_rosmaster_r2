# ROSMASTER R2 adding FTG to pure pursuit to use as f1tenth car 
## Pure Pursuit + Automatic FTG Obstacle Avoidance Continued from Pure Pursuit added Repo 
### (AMCL • Raceline • Safety Mux • Deadman Enabled)

---

## What this system does (plain English)

Your ROSMASTER R2 normally follows a **recorded raceline** using **Pure Pursuit (PP)** and **AMCL localization**.

This setup adds **Follow-The-Gap (FTG)** so that:

- When the path ahead is clear → **Pure Pursuit drives**
- When an obstacle appears → **FTG automatically takes over**
- FTG **steers the car around the obstacle**
- Once clear → **Pure Pursuit automatically resumes**
- You must **hold the enable button** (deadman) for motion
- You can **E-STOP anytime**
- Added Status Light program to show if PP, FTG or manual control

You do **not** manually switch modes.  
The system decides automatically and safely.

---

## What is already working (DO NOT CHANGE)

These parts are **locked and preserved**:

- AMCL localization (map → odom)
- Raceline YAML → `/raceline_path`
- Pure Pursuit publishing to `/cmd_vel_auto_raw`
- Deadman button publishing `/pp_enable`
- `cmdvel_gate.py` gating PP output
- `twist_mux` owning `/cmd_vel`
- Teleop and E-STOP behavior

⚠️ **Do NOT modify Pure Pursuit code.**

---

## How obstacle avoidance works (simple)

1. **Pure Pursuit** drives the raceline  
2. **FTG** watches the LiDAR scan  
3. If an obstacle gets too close:
   - FTG finds the **largest open gap**
   - FTG aims the car **through that gap**
4. FTG publishes a **safety command**
5. `twist_mux` automatically gives FTG priority
6. Once the obstacle is gone:
   - FTG stops publishing
   - `twist_mux` gives control back to Pure Pursuit
7. status_light
   Blue (FTG) relies on /ftg_auto_switch/active (we added that in the debug version).
   Green (PP) uses /pp_enable + “recent /cmd_vel_auto”.
   Yellow (Manual) watches /cmd_vel_teleop and only turns yellow if the command is nonzero recently.
   Gray = nothing happening.

---

## Folder overview (what was added)

```
yahboomcar_ws/src/
├── r2_ftg_safety/
│   ├── scripts/
│   │   ├── ftg_f1tenth_node.py
│   │   └── ftg_auto_switch_node.py
│   ├── config/
│   │   ├── ftg_f1tenth.yaml
│   │   └── ftg_auto_switch.yaml
│
├── r2_estop_joy/
│   └── launch/
│       └── safety_mux_f1tenth.launch
```

Your original `safety_mux.launch` is **not removed**.

---

## Safety rules (IMPORTANT)

- 🚫 Car will NOT move unless enable button is held
- 🛑 E-STOP always overrides everything
- 🧠 Only **one node** publishes `/cmd_vel_safety`
- 🤖 FTG never bypasses the safety system

---

## Bring-up order (FAST / RACE DAY)

### Terminal 1 — LiDAR
```bash
roslaunch yahboomcar_nav laser_bringup.launch
```

### Terminal 2 — Localization (AMCL only)
```bash
roslaunch r2_amcl_localization amcl_only.launch map:=/home/jetson/maps/home2.yaml
```

### Terminal 3 — Raceline Path
```bash
roslaunch r2_raceline_pp raceline_to_path.launch raceline:=/home/jetson/paths/raceline_amcl_01.yaml
```

### Terminal 4 — Safety + FTG (AUTOMATIC)
```bash
roslaunch r2_estop_joy safety_mux_f1tenth.launch mode:=switch
```

### Terminal 5 — Pure Pursuit
```bash
roslaunch r2_raceline_pp pure_pursuit.launch raceline:=/home/jetson/paths/raceline_amcl_01.yaml
```

### Terminal 6 — RViz
```bash
roslaunch r2_raceline_pp amcl_raceline_rviz_fixed.launch
```
### Terminal 7 — Status Light
```bash
rosrun r2_estop_joy r2_status_light.py
```

---

## How to drive

1. Place robot on map
2. Confirm AMCL pose looks correct
3. Hold **ENABLE button (A / R1)**
4. Robot follows raceline
5. Place obstacle:
   - Robot drives **around** it
6. Remove obstacle:
   - Robot returns to raceline

---

## FTG modes

### Default (recommended)
```bash
mode:=switch
```
FTG fully takes control when needed.

### Optional smooth mode
```bash
mode:=blend
```
FTG blends steering with Pure Pursuit.

---

## How to confirm FTG is working

### FTG output
```bash
rostopic echo /cmd_vel_ftg_raw
```

### Safety takeover
```bash
rostopic echo /cmd_vel_safety
```

### Final motor command
```bash
rostopic echo /cmd_vel
```

---

## Build commands

```bash
cd ~/yahboomcar_ws
catkin_make
source devel/setup.bash
```

---

## Summary

- Pure Pursuit unchanged
- AMCL unchanged
- Raceline unchanged
- Safety preserved
- FTG guides car around obstacles
- Automatic takeover + release
- Deadman required
- E-STOP always works
- GitHub safe
