<h1 align="center">
            ✨ 6 Axis Robotic Arm ✨
</h1>

<div align="center">

![Badge](https://img.shields.io/badge/Tech_Stack-python-yellow) ![Badge](https://img.shields.io/badge/Type-OpenSource-orange) ![Badge](https://img.shields.io/badge/For-Students-red)

</div>

<br />

## Overview 🔨

A Raspberry Pi 4 robotic arm project with **camera-based object detection**. A fixed side-mounted camera detects objects by colour, estimates their distance, and moves the arm to touch them using inverse kinematics.

- 5 working servos: base, shoulder, elbow, lower wrist, wrist
- Servo channels: **1, 2, 13, 14, 15** on a 16-channel PCA9685 board
- Pulse width range: **1000–2000 µs** (confirmed from servo tests)

<br />

## Tech Stack

- **Language:** Python 3
- **Libraries:** OpenCV, adafruit-servokit, NumPy
- **Hardware:** Raspberry Pi 4, PCA9685 servo driver (I2C), USB/Pi camera
- **Version Control:** Git and GitHub

<br />

## Gallery

<a href='https://youtube.com/shorts/JnGvtVqmeKE?si=3DLGpbV85BaUr5AF'>Click here for video of the robotic arm</a>

<br />

## Files

| File | Description |
|---|---|
| `arm_vision_control.py` | **Main script** — camera detects object, arm moves to touch it |
| `color_tuner.py` | Helper to tune colour detection for your target object |
| `test/testallservos2.py` | Test individual servos by channel number |
| `test/testallservos.py` | Continuous servo test |
| `one.py` | Basic single-servo test |

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

### 3. Tune colour detection for your object

Run the colour tuner and adjust the sliders until only your target object appears white:

```bash
python color_tuner.py
```

Press **S** to print the values, then copy them into `arm_vision_control.py` under `COLOR_LOWER1 / COLOR_UPPER1`.

### 4. Configure the camera mounting position

Open `arm_vision_control.py` and fill in the **CAMERA MOUNTING** section near the top.
Measure from your robot base to where the camera is physically sitting:

```python
CAM_X =   0.0    # cm forward/back from robot base
CAM_Y = -30.0    # cm left(+) or right(-) from robot base
CAM_Z =  20.0    # cm above the table

CAM_PAN_DEG  = 90.0   # direction camera faces (90 = facing left, toward arm)
CAM_TILT_DEG = 15.0   # degrees tilted downward
```

**Quick guide for CAM_PAN_DEG:**
- Camera on the **right** side, facing the arm → `90`
- Camera on the **left** side, facing the arm → `-90`
- Camera **behind** the arm, facing forward → `0`
- Camera **in front** of the arm, facing it → `180`

### 5. Set your arm dimensions

Measure your arm's physical link lengths in cm and update:

```python
L1 = 10.5   # shoulder → elbow (cm)
L2 = 10.0   # elbow → lower wrist (cm)
L3 = 5.5    # lower wrist → claw tip (cm)
```

### 6. Set the object width

Set the real-world width of the object you want to grab:

```python
KNOWN_OBJECT_WIDTH_CM = 6.0   # measure your object
```

<br />

## Run — Vision Guided Mode

```bash
python arm_vision_control.py
```

A camera window will open showing the live feed.

| Key | Action |
|---|---|
| **G** | Grab — moves arm to touch the detected object |
| **Q** | Quit |

The HUD shows the detected object's depth, and the calculated servo angles in real time. When an object is detected and within reach, the status shows **"Ready — press G"**.

<br />

## Run — Servo Tests

Test individual servos to verify wiring before running the vision script:

```bash
python test/testallservos2.py
```

Type a channel number (0–15) and press Enter to move that servo from 0° to 180°.

<br />

## Troubleshooting

| Problem | Fix |
|---|---|
| Arm moves in the wrong direction | Swap `CAM_PAN_DEG` by ±90° |
| Left/right is mirrored on screen | Set `CAM_FLIP_IMAGE_X = True` |
| Object not detected | Re-run `color_tuner.py` and update colour values |
| "Out of reach" shown | Move the object closer, or increase `L1`/`L2` if your arm is longer |
| Servos not moving | Check I2C wiring and run `test/testallservos2.py` first |

<br />

[![Uses Git](https://forthebadge.com/images/badges/uses-git.svg)](https://github.com/hokocodes/robotic-arm-raspian)
[![Built with love](https://forthebadge.com/images/badges/built-by-developers.svg)](https://github.com/hokocodes/robotic-arm-raspian)
