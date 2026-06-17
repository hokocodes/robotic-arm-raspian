#!/usr/bin/env python3
"""
Vision-guided robotic arm controller
=====================================
Uses a FIXED side-mounted camera and depth-from-size estimation to locate
an object in 3-D space, then drives 5 servos (base, shoulder, elbow,
lower_wrist, wrist) to touch it with the claw.

The camera is NOT on the arm — it sits at a fixed position to the side
of the robot. Its view is transformed into the robot's coordinate frame
using the camera mounting position and orientation you configure below.

Hardware
--------
- Raspberry Pi 4
- PCA9685 servo driver via I2C
- USB or Pi camera mounted to the side (fixed, not on the arm)
- 5 servos on channels 0-4; optional claw on channel 5

Quick start
-----------
1. Measure and fill in the CAMERA MOUNTING section below:
     - Where the camera sits relative to the robot base (CAM_X/Y/Z)
     - Which direction it faces (CAM_PAN_DEG)
     - How much it tilts downward (CAM_TILT_DEG)
2. Tune the HSV color range for your object:
       python color_tuner.py
3. Set KNOWN_OBJECT_WIDTH_CM to the real width of your target object.
4. Measure arm segment lengths and set L1, L2, L3.
5. Run:
       python arm_vision_control.py
   Press G to grab the detected object, Q to quit.

Camera mounting diagram (top-down view, arm faces +X):
                        +X (forward)
                         ^
                         |
              [-Y] ──── base ──── [+Y]

  Example — camera on the RIGHT side, facing left toward the arm:
    CAM_X =   0    (level with base, not forward or back)
    CAM_Y = -30    (30 cm to the right)
    CAM_Z =  20    (20 cm above the table)
    CAM_PAN_DEG  =  90   (faces +Y, i.e. toward the arm workspace)
    CAM_TILT_DEG =  15   (tilted 15° downward)

  Example — camera behind the arm, facing forward:
    CAM_X = -20, CAM_Y = 0, CAM_Z = 25
    CAM_PAN_DEG = 0, CAM_TILT_DEG = 20
"""

import math
import time

import cv2
import numpy as np

# ── Hardware (PCA9685) ────────────────────────────────────────────────────────
try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685

    _i2c = busio.I2C(board.SCL, board.SDA)
    _pca = PCA9685(_i2c)
    _pca.frequency = 50
    HARDWARE = True
    print("[INFO] PCA9685 found — servos active.")
except Exception as _e:
    HARDWARE = False
    _pca = None
    print(f"[WARN] PCA9685 not available ({_e}). Running in simulation mode.")


# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION — edit these to match your robot
# ═══════════════════════════════════════════════════════════════════════════════

# PCA9685 channel for each joint (set to None if not connected)
SERVO_CHANNEL = {
    "base":        0,   # rotates the whole arm left/right
    "shoulder":    1,   # raises/lowers the upper arm
    "elbow":       2,   # bends the forearm
    "lower_wrist": 3,   # rolls the wrist
    "wrist":       4,   # tilts the claw up/down
    "claw":        5,   # open / close  (None if your 6th servo is broken)
}

# Physical arm link lengths in centimetres — measure yours!
L1 = 10.5   # shoulder pivot → elbow pivot
L2 = 10.0   # elbow pivot   → lower-wrist pivot
L3 = 5.5    # lower-wrist   → claw tip

# Safe angle limits [min°, max°]
LIMITS = {
    "base":        (0,   180),
    "shoulder":    (30,  150),
    "elbow":       (0,   150),
    "lower_wrist": (0,   180),
    "wrist":       (50,  130),
    "claw":        (0,    70),   # 0 = fully open, 70 = closed
}

# Resting / home angles
HOME = {
    "base":        90,
    "shoulder":    90,
    "elbow":       90,
    "lower_wrist": 90,
    "wrist":       90,
    "claw":        0,
}

# ── Camera ───────────────────────────────────────────────────────────────────
CAMERA_INDEX = 0        # 0 = first attached camera
FRAME_W      = 640
FRAME_H      = 480
HFOV_DEG     = 62.0     # horizontal field-of-view (check your camera datasheet)
VFOV_DEG     = 48.0     # vertical field-of-view

# ── Camera mounting (FIXED, side-mounted — NOT on the arm) ───────────────────
# Measure these from your actual setup (all in cm / degrees).
#
# Robot base frame: x = forward, y = left, z = up.
#
# CAM_X/Y/Z — camera lens position relative to the robot base pivot point.
CAM_X =   0.0    # cm forward (+) or back (-) from robot base
CAM_Y = -30.0    # cm left (+) or right (-) from robot base
CAM_Z =  20.0    # cm above the table surface

# CAM_PAN_DEG — horizontal direction the camera faces, measured from
#   the robot's +X (forward) axis, counterclockwise when viewed from above.
#   0°  = camera faces forward (same direction as arm at home)
#   90° = camera faces left  (+Y)     ← typical for right-side mount
#  -90° = camera faces right (-Y)     ← typical for left-side mount
#  180° = camera faces backward (toward the arm from the front)
CAM_PAN_DEG  =  90.0   # degrees — adjust to your setup

# CAM_TILT_DEG — how far the camera is tilted downward from horizontal.
#   0°  = camera looks perfectly level
#  +15° = camera tilts 15° down toward the table (most common)
CAM_TILT_DEG =  15.0   # degrees — adjust to your setup

# CAM_FLIP_IMAGE_X — set True if the left/right direction appears reversed
#   in the camera feed (e.g. if a mirror or upside-down mount is used).
CAM_FLIP_IMAGE_X = False

# ── Depth-from-apparent-size ──────────────────────────────────────────────────
# Measure the real width of the object you want to grab (cm)
KNOWN_OBJECT_WIDTH_CM = 6.0

# ── HSV colour for the target object (default = red) ─────────────────────────
# Use color_tuner.py to find the right values for your object.
COLOR_LOWER1 = np.array([0,   120,  70])   # red wraps around 0/180 in OpenCV HSV
COLOR_UPPER1 = np.array([10,  255, 255])
COLOR_LOWER2 = np.array([170, 120,  70])
COLOR_UPPER2 = np.array([180, 255, 255])
MIN_BLOB_AREA = 600     # px² — ignore blobs smaller than this

# ── Motion ────────────────────────────────────────────────────────────────────
SMOOTH_STEPS  = 10       # interpolation steps when moving arm
STEP_DELAY    = 0.06     # seconds between steps
HOLD_SECONDS  = 1.0      # pause at target before retreating

# ═══════════════════════════════════════════════════════════════════════════════
# SERVO DRIVER
# ═══════════════════════════════════════════════════════════════════════════════

# Track current angles so we can interpolate
_state: dict[str, float] = dict(HOME)


def _angle_to_duty(angle: float) -> int:
    """Convert 0-180° to PCA9685 16-bit duty cycle.

    Standard servo: 500 µs = 0°, 2500 µs = 180°, period = 20 000 µs.
    """
    pulse_us = 500 + (angle / 180.0) * 2000
    return int(min(max(pulse_us / 20_000 * 65535, 0), 65535))


def _set_servo_raw(name: str, angle: float) -> None:
    """Write one servo angle (clamped to limits) immediately."""
    lo, hi = LIMITS[name]
    angle = max(lo, min(hi, angle))
    _state[name] = angle
    ch = SERVO_CHANNEL.get(name)
    if ch is None:
        return
    if HARDWARE:
        _pca.channels[ch].duty_cycle = _angle_to_duty(angle)
    else:
        print(f"  [SIM] {name:12s} → {angle:6.1f}°")


def move_home() -> None:
    print("[ARM] Moving to home position…")
    for name, angle in HOME.items():
        _set_servo_raw(name, angle)
    time.sleep(0.8)


def smooth_move(targets: dict[str, float]) -> None:
    """Interpolate every joint from current position to target over SMOOTH_STEPS."""
    starts = {k: _state.get(k, HOME.get(k, 90)) for k in targets}
    for step in range(1, SMOOTH_STEPS + 1):
        t = step / SMOOTH_STEPS
        for name, target in targets.items():
            _set_servo_raw(name, starts[name] + (target - starts[name]) * t)
        time.sleep(STEP_DELAY)


# ═══════════════════════════════════════════════════════════════════════════════
# INVERSE KINEMATICS
# ═══════════════════════════════════════════════════════════════════════════════

def ik_solve(x: float, y: float, z: float) -> dict[str, float] | None:
    """Compute joint angles to place the claw at (x, y, z) cm in robot frame.

    Robot frame convention (arm at home position pointing forward):
      x = forward (away from base)
      y = left  (positive = left when facing the arm)
      z = up    (positive = upward)

    Returns a dict of joint angles (degrees) or None if the target is
    unreachable or would exceed joint limits.
    """
    # ── Base (yaw) ────────────────────────────────────────────────────────────
    base_deg = math.degrees(math.atan2(y, x)) + 90.0   # 90° offset → forward = 90°

    # ── Planar 2-link IK in the sagittal plane ────────────────────────────────
    reach = math.sqrt(x**2 + y**2)   # horizontal reach
    r     = math.sqrt(reach**2 + z**2)

    max_r = (L1 + L2) * 0.98         # stay slightly inside full extension
    if r > max_r:
        scale = max_r / r
        reach *= scale
        z     *= scale
        r      = max_r

    cos_elbow = (r**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow_rad  = math.acos(cos_elbow)

    alpha = math.atan2(z, reach)
    beta  = math.atan2(L2 * math.sin(elbow_rad),
                       L1 + L2 * math.cos(elbow_rad))
    shoulder_rad = alpha - beta

    # Map to servo convention: 90° = arm horizontal, 150° = arm up, 30° = arm down
    shoulder_deg = 90.0 - math.degrees(shoulder_rad)
    # 90° = elbow straight, larger = more bent
    elbow_deg    = 180.0 - math.degrees(elbow_rad)

    # ── Wrist: keep claw level with the ground ────────────────────────────────
    wrist_deg = 90.0 + math.degrees(shoulder_rad) + (math.degrees(elbow_rad) - 180.0)
    wrist_deg = max(LIMITS["wrist"][0], min(LIMITS["wrist"][1], wrist_deg))

    angles = {
        "base":        base_deg,
        "shoulder":    shoulder_deg,
        "elbow":       elbow_deg,
        "lower_wrist": 90.0,     # keep wrist roll neutral
        "wrist":       wrist_deg,
    }

    # ── Limit check ───────────────────────────────────────────────────────────
    for name, angle in angles.items():
        lo, hi = LIMITS[name]
        if not (lo - 2 <= angle <= hi + 2):
            print(f"[IK ] {name} = {angle:.1f}° exceeds limits [{lo}°, {hi}°] — skip")
            return None

    return angles


# ═══════════════════════════════════════════════════════════════════════════════
# VISION — object detection and depth estimation
# ═══════════════════════════════════════════════════════════════════════════════

_focal_px: float | None = None   # computed once from HFOV


def _get_focal() -> float:
    global _focal_px
    if _focal_px is None:
        _focal_px = FRAME_W / (2.0 * math.tan(math.radians(HFOV_DEG / 2.0)))
    return _focal_px


def detect_object(frame: np.ndarray) -> tuple[int, int, int] | None:
    """Detect the target object by HSV colour.

    Returns (cx, cy, width_px) of the largest matching blob, or None.
    cx/cy are pixel coordinates; width_px is the bounding-box width.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = (cv2.inRange(hsv, COLOR_LOWER1, COLOR_UPPER1) |
            cv2.inRange(hsv, COLOR_LOWER2, COLOR_UPPER2))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    biggest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(biggest) < MIN_BLOB_AREA:
        return None

    bx, by, bw, bh = cv2.boundingRect(biggest)
    return bx + bw // 2, by + bh // 2, bw


def pixel_to_robot_frame(cx: int, cy: int, width_px: int) -> tuple[float, float, float]:
    """Convert a pixel detection into a 3-D position in the robot base frame (cm).

    The camera is FIXED to the side — not on the arm.  This function does
    three things:
      1. Depth estimation  — pinhole formula using known object width.
      2. Ray casting       — pixel offset → 3-D point in the CAMERA frame.
      3. Coordinate transform — rotate + translate camera frame into the
                                ROBOT base frame using the mounting config.

    Camera frame convention used here:
      +x_cam = right in the image
      +y_cam = up   in the image
      +z_cam = depth (into the scene, away from the lens)

    Robot base frame:
      +x = forward  (arm points this way at home)
      +y = left
      +z = up

    The rotation from camera frame → robot frame is:
      R = Rz(pan) · Ry(-tilt)          (pan around world Z, then tilt around
                                         the camera's own horizontal axis)

    Translation: add the camera mounting position (CAM_X, CAM_Y, CAM_Z).
    """
    f = _get_focal()

    # ── 1. Depth from apparent size ───────────────────────────────────────────
    depth_cm = (KNOWN_OBJECT_WIDTH_CM * f) / max(width_px, 1)

    # ── 2. Pixel offsets → angles → point in CAMERA frame ────────────────────
    px_x = (cx - FRAME_W / 2.0) * (-1 if CAM_FLIP_IMAGE_X else 1)
    px_y = -(cy - FRAME_H / 2.0)   # flip: pixels grow down, y_cam grows up

    angle_h = math.atan2(px_x, f)   # horizontal ray angle in camera frame
    angle_v = math.atan2(px_y, f)   # vertical   ray angle in camera frame

    # 3-D point in camera frame (x=right, y=up, z=forward)
    xc = depth_cm * math.cos(angle_v) * math.sin(angle_h)
    yc = depth_cm * math.sin(angle_v)
    zc = depth_cm * math.cos(angle_v) * math.cos(angle_h)

    # Aim for top surface of object rather than its centre
    yc += KNOWN_OBJECT_WIDTH_CM / 2.0

    # ── 3. Camera frame → Robot base frame ───────────────────────────────────
    # Tilt: rotate around camera x-axis by TILT degrees (positive = nose down).
    # Ry(tilt) acting on the yz plane:
    tilt = math.radians(CAM_TILT_DEG)
    ct, st = math.cos(tilt), math.sin(tilt)
    #   y_t = y*cos(tilt) - z*sin(tilt)
    #   z_t = y*sin(tilt) + z*cos(tilt)
    xt =  xc
    yt =  yc * ct - zc * st
    zt =  yc * st + zc * ct

    # Pan: rotate around world Z-axis by PAN degrees.
    # After pan, the camera's +z_cam axis points in the pan direction.
    # Robot frame:   x_r = zt*cos(pan) - xt*sin(pan)
    #                y_r = zt*sin(pan) + xt*cos(pan)
    #                z_r = yt
    pan = math.radians(CAM_PAN_DEG)
    cp, sp = math.cos(pan), math.sin(pan)

    x_robot = zt * cp - xt * sp
    y_robot = zt * sp + xt * cp
    z_robot = yt

    # Translate: add camera mounting offset
    x_robot += CAM_X
    y_robot += CAM_Y
    z_robot += CAM_Z

    return x_robot, y_robot, z_robot


# ═══════════════════════════════════════════════════════════════════════════════
# GRAB SEQUENCE
# ═══════════════════════════════════════════════════════════════════════════════

def grab_sequence(angles: dict[str, float]) -> None:
    """Move to target → open claw → advance → close claw → retreat home."""
    print("[ARM] Opening claw…")
    _set_servo_raw("claw", LIMITS["claw"][0])   # fully open
    time.sleep(0.3)

    print("[ARM] Moving to target…")
    smooth_move(angles)
    time.sleep(HOLD_SECONDS)

    print("[ARM] Closing claw (touching object)…")
    _set_servo_raw("claw", LIMITS["claw"][1])   # close
    time.sleep(0.6)

    print("[ARM] Holding…")
    time.sleep(HOLD_SECONDS)

    print("[ARM] Returning home…")
    _set_servo_raw("claw", LIMITS["claw"][0])   # open to release
    time.sleep(0.3)
    move_home()


# ═══════════════════════════════════════════════════════════════════════════════
# HUD OVERLAY
# ═══════════════════════════════════════════════════════════════════════════════

def draw_hud(frame: np.ndarray,
             cx: int, cy: int, width_px: int,
             depth_cm: float,
             angles: dict | None,
             status: str) -> None:
    h, w = frame.shape[:2]
    # crosshair at image centre
    cv2.line(frame, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (200, 200, 200), 1)
    cv2.line(frame, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (200, 200, 200), 1)

    # detection marker
    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 2)
    cv2.drawMarker(frame, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

    # info panel background
    cv2.rectangle(frame, (0, 0), (250, 175), (0, 0, 0), -1)
    cv2.rectangle(frame, (0, 0), (250, 175), (60, 60, 60), 1)

    def txt(text, row, colour=(220, 220, 220)):
        cv2.putText(frame, text, (8, 20 + row * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, colour, 1, cv2.LINE_AA)

    txt(f"Status : {status}", 0, (0, 220, 100) if angles else (80, 80, 255))
    txt(f"Depth  : {depth_cm:.1f} cm", 1)
    if angles:
        txt(f"Base   : {angles['base']:.0f}°",     2)
        txt(f"Shldr  : {angles['shoulder']:.0f}°", 3)
        txt(f"Elbow  : {angles['elbow']:.0f}°",    4)
        txt(f"Wrist  : {angles['wrist']:.0f}°",    5)
    txt("G=grab  Q=quit", 7, (180, 180, 100))


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    move_home()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        raise RuntimeError(
            f"Could not open camera index {CAMERA_INDEX}. "
            "Try changing CAMERA_INDEX (0, 1, 2…)."
        )

    print("[VIS] Camera ready.  G = grab   Q = quit")

    last_angles: dict | None = None
    last_depth:  float        = 0.0
    last_cx:     int          = FRAME_W // 2
    last_cy:     int          = FRAME_H // 2
    last_wpx:    int          = 1
    status = "Searching…"

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Frame read failed — retrying…")
                time.sleep(0.05)
                continue

            detection = detect_object(frame)

            if detection is not None:
                cx, cy, wpx = detection
                x, y, z     = pixel_to_robot_frame(cx, cy, wpx)
                depth_cm    = math.sqrt(x**2 + y**2 + z**2)
                angles      = ik_solve(x, y, z)

                last_cx, last_cy, last_wpx = cx, cy, wpx
                last_depth  = depth_cm
                last_angles = angles
                status = "Ready — press G" if angles else "Out of reach"
            else:
                status = "Searching…"
                last_angles = None

            draw_hud(frame, last_cx, last_cy, last_wpx, last_depth, last_angles, status)
            cv2.imshow("Robotic Arm — Vision Control", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('g'):
                if last_angles:
                    cap.release()
                    cv2.destroyAllWindows()
                    grab_sequence(last_angles)
                    # Reopen camera after grab
                    cap = cv2.VideoCapture(CAMERA_INDEX)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
                    last_angles = None
                else:
                    print("[WARN] No valid target — cannot grab.")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        move_home()
        if HARDWARE and _pca is not None:
            _pca.deinit()
        print("[INFO] Shut down cleanly.")


if __name__ == "__main__":
    main()
