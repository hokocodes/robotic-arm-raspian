<h1 align="center">
            ✨ 6 Axis Robotic Arm ✨
</h1>

<div align="center">

![Badge](https://img.shields.io/badge/Tech_Stack-python-yellow) ![Badge](https://img.shields.io/badge/Type-OpenSource-orange) ![Badge](https://img.shields.io/badge/For-Students-red)

</div>

<br />

## Overview 🔨

A Raspberry Pi 4 robotic arm with **click-to-aim camera control**.

A fixed camera sits to the **left of the arm**, looking at the workspace.
Click anywhere on the live camera feed — the arm immediately moves to that spot.
No colour detection, no calibration objects needed.

- 5 working servos: base, shoulder, elbow, lower wrist, wrist
- Servo channels: **1, 2, 13, 14, 15** on a 16-channel PCA9685 board
- Pulse width range: **1000–2000 µs**

<br />

## Files

| File | Description |
|---|---|
| `arm_vision_control.py` | **Main script** — live video, click to aim, arm follows |
| `color_tuner.py` | HSV colour tuner (only needed if you add colour detection later) |
| `test/testallservos2.py` | Test individual servos by channel number |
| `test/testallservos.py` | Continuous servo test |
| `one.py` | Basic single-servo test |

<br />

## How it works

```
You click a point on the camera video
         ↓
A ray is cast from the camera through that pixel
         ↓
The ray intersects the table surface (at TARGET_Z_CM height)
         ↓
Inverse kinematics converts that 3-D point to servo angles
         ↓
The arm smoothly moves there (video keeps playing)
```

No object-size calibration or colour setup required.

<br />

## Setup

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Wire your servos

Connect each servo to the PCA9685 board on these channels:

| Joint | Channel |
|---|---|
| Base (rotates left/right) | 1 |
| Shoulder (raises/lowers) | 2 |
| Elbow (bends forearm) | 13 |
| Lower wrist (rolls) | 14 |
| Wrist (tilts claw) | 15 |

### 3. Mount the camera

Place the camera to the **left of the robot base**, facing right toward the workspace.

```
  TOP VIEW

  +X (forward — arm points this way)
   ^
   |
[camera] ──── robot base ────
(left side)
```

### 4. Measure and set your camera position

Open `arm_vision_control.py` and update the mounting values near the top:

```python
CAM_X =   0.0    # cm forward/back from robot base (0 = level with base)
CAM_Y =  30.0    # cm to the LEFT of the robot base  ← measure this
CAM_Z =  20.0    # cm above the table                ← measure this
```

Leave `CAM_PAN_DEG = -90.0` and `CAM_TILT_DEG = 15.0` — these are correct for a
left-side camera facing the workspace. Adjust tilt if your camera is more/less angled.

### 5. Set target height

```python
TARGET_Z_CM = 3.0   # cm above the table the arm aims for
                    # 0 = table surface, 3 = top of a small object
```

### 6. Set your arm link lengths

Measure each segment of your arm (in cm) and update:

```python
L1 = 10.5   # shoulder pivot → elbow pivot
L2 = 10.0   # elbow pivot   → lower-wrist pivot
L3 = 5.5    # lower-wrist   → claw tip
```

<br />

## Run

```bash
python arm_vision_control.py
```

A live camera window opens.

| Control | Action |
|---|---|
| **Left-click** on video | Aim arm at that point — arm moves immediately |
| **G** | Grab sequence: open claw → move to target → close → retreat home |
| **H** | Return arm to home position |
| **Q** | Quit |

The HUD shows the current status, the computed 3-D position, and all servo angles in real time.

<br />

## Servo Tests

Before running the main script, verify your wiring with the servo tester:

```bash
python test/testallservos2.py
```

Type a channel number (0–15) and press Enter to sweep that servo from 0° → 180°.

<br />

## Troubleshooting

| Problem | Fix |
|---|---|
| Arm moves in wrong direction (left/right swapped) | Set `CAM_FLIP_IMAGE_X = True` |
| Arm reaches too far or not far enough | Adjust `CAM_Y` / `CAM_Z` to match your actual camera position |
| "Ray missed table" shown on click | Click lower in the image (ray is going above the table plane) |
| "Out of reach" shown | Click closer to the arm base, or check that `L1`/`L2` match your arm |
| Arm aims too high/low | Adjust `TARGET_Z_CM` (higher value = arm aims higher above table) |
| Wrong camera opens | Change `CAMERA_INDEX` from `0` to `1` or `2` |
| Servos not moving | Run `test/testallservos2.py` to verify wiring first |

<br />

## Gallery

<a href='https://youtube.com/shorts/JnGvtVqmeKE?si=3DLGpbV85BaUr5AF'>Click here for video of the robotic arm</a>

<br />

[![Uses Git](https://forthebadge.com/images/badges/uses-git.svg)](https://github.com/hokocodes/robotic-arm-raspian)
[![Built with love](https://forthebadge.com/images/badges/built-by-developers.svg)](https://github.com/hokocodes/robotic-arm-raspian)
