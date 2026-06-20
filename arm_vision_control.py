#!/usr/bin/env python3
"""
Click-to-target robotic arm controller
=======================================
The camera is FIXED to the LEFT of the robot arm, facing the workspace.

How to use
----------
1. Run:  python arm_vision_control.py
2. A live camera window opens.
3. LEFT-CLICK anywhere on the video to aim the arm at that spot.
   The arm moves there immediately (video keeps playing).
4. Click again to redirect the arm to a new spot.

Keys
----
  Left-click  — aim arm at clicked point
  G           — grab sequence (open claw → move → close → retreat home)
  H           — return arm to home position
  Q           — quit

Depth is computed by casting a ray from the camera through the clicked pixel
and finding where it hits the table surface (z = TARGET_Z_CM).  No colour
detection or object size needed.

Hardware
--------
  Raspberry Pi 4
  PCA9685 16-channel servo driver (I2C) via adafruit_servokit
  USB or Pi camera, fixed to the LEFT of the robot base
  5 working servos on channels 1, 2, 13, 14, 15
"""

import math
import threading
import time

import cv2
import numpy as np

# ── Hardware (ServoKit) ───────────────────────────────────────────────────────
try:
    from adafruit_servokit import ServoKit

    _kit = ServoKit(channels=16)
    for _ch in [1, 2, 13, 14, 15]:
        _kit.servo[_ch].set_pulse_width_range(1000, 2000)

    HARDWARE = True
    print("[INFO] ServoKit ready — channels 1, 2, 13, 14, 15.")
except Exception as _e:
    HARDWARE = False
    _kit = None
    print(f"[WARN] ServoKit not available ({_e}). Simulation mode.")


# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION — measure and adjust these to match your physical setup
# ═══════════════════════════════════════════════════════════════════════════════

# Servo channel for each joint (confirmed from test/testallservos2.py)
SERVO_CHANNEL = {
    "base":        1,    # rotates the whole arm left/right
    "shoulder":    2,    # raises/lowers the upper arm
    "elbow":       13,   # bends the forearm
    "lower_wrist": 14,   # rolls the wrist
    "wrist":       15,   # tilts the claw up/down
    "claw":        None, # not working — set a channel number if it is
}

# Physical arm link lengths in centimetres — measure your arm!
L1 = 10.5   # shoulder pivot → elbow pivot
L2 = 10.0   # elbow pivot   → lower-wrist pivot
L3 = 5.5    # lower-wrist   → claw tip

# Safe angle limits per joint  [min°, max°]
LIMITS = {
    "base":        (0,   180),
    "shoulder":    (30,  150),
    "elbow":       (0,   150),
    "lower_wrist": (0,   180),
    "wrist":       (50,  130),
    "claw":        (0,    70),  # 0 = open, 70 = closed
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

# ── Camera ────────────────────────────────────────────────────────────────────
CAMERA_INDEX     = 0      # 0 = first USB/Pi camera; try 1 or 2 if wrong camera opens
FRAME_W          = 640
FRAME_H          = 480
HFOV_DEG         = 62.0   # horizontal field-of-view of your camera (check datasheet)
CAM_FLIP_IMAGE_X = False  # set True if left/right appears mirrored in the video

# ── Camera mounting position (robot frame: +X=forward, +Y=left, +Z=up) ───────
#
# Camera is on the LEFT of the robot, facing RIGHT toward the workspace.
#
#              TOP VIEW
#
#              +X (forward / arm points here)
#               ^
#   [camera] ──┤──── robot base ────
#   (left, +Y) |
#
# Measure from the robot base pivot to where the camera lens is (in cm).
CAM_X =   0.0    # flush with the base (0 = not forward or back)
CAM_Y =  30.0    # 30 cm to the LEFT  ← adjust to your actual distance
CAM_Z =  20.0    # 20 cm above the table surface ← adjust to actual height

# Camera faces RIGHT (–Y direction) to look into the workspace.
# –90° means "rotated 90° clockwise from +X axis when viewed from above" = faces –Y.
CAM_PAN_DEG  = -90.0   # do NOT change unless your camera faces a different way
CAM_TILT_DEG =  15.0   # degrees tilted downward (0 = perfectly level)

# ── Target height ─────────────────────────────────────────────────────────────
# The script intersects the camera ray with a horizontal plane at this height.
# Set to the height of the top surface of your objects above the table.
#   0.0 = aim at the table surface itself
#   3.0 = aim 3 cm above the table (top of a small object sitting on the table)
TARGET_Z_CM = 3.0

# ── Motion smoothing ──────────────────────────────────────────────────────────
SMOOTH_STEPS = 12      # more steps = slower but smoother movement
STEP_DELAY   = 0.05   # seconds between each interpolation step
HOLD_SECONDS = 1.0    # how long to hold position after reaching target


# ═══════════════════════════════════════════════════════════════════════════════
# SERVO DRIVER
# ═══════════════════════════════════════════════════════════════════════════════

_state: dict[str, float] = dict(HOME)


def _set_servo_raw(name: str, angle: float) -> None:
    """Write one servo angle (clamped to limits) immediately."""
    lo, hi = LIMITS[name]
    angle = max(lo, min(hi, angle))
    _state[name] = angle
    ch = SERVO_CHANNEL.get(name)
    if ch is None:
        return
    if HARDWARE:
        _kit.servo[ch].angle = angle
    else:
        print(f"  [SIM] {name:12s} ch={ch!s:>3}  → {angle:6.1f}°")


def move_home() -> None:
    print("[ARM] Moving to home position…")
    for name, angle in HOME.items():
        _set_servo_raw(name, angle)
    time.sleep(0.8)


def smooth_move(targets: dict[str, float]) -> None:
    """Interpolate every joint from current position to target."""
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
    """Compute joint angles to reach (x, y, z) cm in robot base frame.

    Robot frame: +x=forward, +y=left, +z=up.
    Returns joint angles dict or None if target is unreachable.
    """
    # Base (yaw): atan2 gives angle from +X axis; 90° offset puts 90° = forward
    base_deg = math.degrees(math.atan2(y, x)) + 90.0

    # 2-link planar IK in the vertical sagittal plane
    reach = math.sqrt(x ** 2 + y ** 2)
    r     = math.sqrt(reach ** 2 + z ** 2)

    # Scale back if beyond max reach (keeps arm as close as possible)
    max_r = (L1 + L2) * 0.98
    if r > max_r:
        scale = max_r / r
        reach *= scale
        z     *= scale
        r      = max_r

    cos_elbow = (r ** 2 - L1 ** 2 - L2 ** 2) / (2.0 * L1 * L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow_rad  = math.acos(cos_elbow)

    alpha        = math.atan2(z, reach)
    beta         = math.atan2(L2 * math.sin(elbow_rad),
                              L1 + L2 * math.cos(elbow_rad))
    shoulder_rad = alpha - beta

    shoulder_deg = 90.0 - math.degrees(shoulder_rad)
    elbow_deg    = 180.0 - math.degrees(elbow_rad)

    # Keep wrist level with the ground
    wrist_deg = (90.0
                 + math.degrees(shoulder_rad)
                 + (math.degrees(elbow_rad) - 180.0))
    wrist_deg = max(LIMITS["wrist"][0], min(LIMITS["wrist"][1], wrist_deg))

    angles = {
        "base":        base_deg,
        "shoulder":    shoulder_deg,
        "elbow":       elbow_deg,
        "lower_wrist": 90.0,   # keep wrist roll neutral
        "wrist":       wrist_deg,
    }

    # Reject if any joint would exceed its limits
    for name, angle in angles.items():
        lo, hi = LIMITS[name]
        if not (lo - 2 <= angle <= hi + 2):
            print(f"[IK ] {name} = {angle:.1f}° out of range [{lo}°,{hi}°]")
            return None

    return angles


# ═══════════════════════════════════════════════════════════════════════════════
# RAY → ROBOT FRAME  (click pixel → world position)
# ═══════════════════════════════════════════════════════════════════════════════

def _focal_px() -> float:
    return FRAME_W / (2.0 * math.tan(math.radians(HFOV_DEG / 2.0)))


def pixel_to_robot_frame(cx: int, cy: int) -> tuple[float, float, float] | None:
    """Cast a ray from the camera through pixel (cx, cy) and find where it hits
    the horizontal plane at z = TARGET_Z_CM in the robot base frame.

    Camera frame convention used here:
      +x_cam = right in the image
      +y_cam = up in the image (flipped from pixel y which grows downward)
      +z_cam = forward (depth, into the scene)

    Steps:
      1. Pixel → ray direction in camera frame.
      2. Rotate ray by camera tilt (around camera x-axis, nose down).
      3. Rotate ray by camera pan (around world z-axis).
      4. Ray origin = camera position (CAM_X, CAM_Y, CAM_Z).
      5. Intersect ray with plane z = TARGET_Z_CM.

    Returns (x, y, z) in robot frame (cm), or None if ray misses the plane.
    """
    f = _focal_px()

    # Pixel offset from image centre
    px_x = (cx - FRAME_W / 2.0) * (-1.0 if CAM_FLIP_IMAGE_X else 1.0)
    px_y = -(cy - FRAME_H / 2.0)  # flip: image y grows down, camera y grows up

    # Ray direction in camera frame (unnormalized, z_cam=1 = forward)
    d = np.array([px_x / f, px_y / f, 1.0])

    # ── Apply tilt (rotate around camera x-axis, positive = nose down) ────
    tilt = math.radians(CAM_TILT_DEG)
    ct, st = math.cos(tilt), math.sin(tilt)
    #   y' = y·cos(tilt) - z·sin(tilt)
    #   z' = y·sin(tilt) + z·cos(tilt)
    d = np.array([d[0],
                  d[1] * ct - d[2] * st,
                  d[1] * st + d[2] * ct])

    # ── Apply pan (rotate around world z-axis) ─────────────────────────────
    # After rotation the camera's +z_cam faces in the pan direction.
    # Robot frame:  x_r = z·cos(pan) - x·sin(pan)
    #               y_r = z·sin(pan) + x·cos(pan)
    #               z_r = y
    pan = math.radians(CAM_PAN_DEG)
    cp, sp = math.cos(pan), math.sin(pan)
    d_robot = np.array([
        d[2] * cp - d[0] * sp,
        d[2] * sp + d[0] * cp,
        d[1],
    ])

    # ── Ray: origin = camera position in robot frame ───────────────────────
    origin = np.array([CAM_X, CAM_Y, CAM_Z])

    # ── Intersect with plane z = TARGET_Z_CM ──────────────────────────────
    dz = d_robot[2]
    if abs(dz) < 1e-6:
        # Ray is nearly horizontal — will never hit the table plane
        return None

    t = (TARGET_Z_CM - origin[2]) / dz
    if t < 0:
        # Intersection is behind the camera
        return None

    point = origin + t * d_robot
    return float(point[0]), float(point[1]), float(point[2])


# ═══════════════════════════════════════════════════════════════════════════════
# HUD OVERLAY
# ═══════════════════════════════════════════════════════════════════════════════

def draw_hud(frame: np.ndarray,
             click_pt: tuple[int, int] | None,
             angles: dict | None,
             robot_pos: tuple | None,
             status: str,
             is_moving: bool) -> None:

    # Marker at the clicked point
    if click_pt:
        cx, cy = click_pt
        colour = (0, 140, 255) if is_moving else (0, 220, 100)
        cv2.circle(frame, (cx, cy), 14, colour, 2)
        cv2.drawMarker(frame, (cx, cy), colour, cv2.MARKER_CROSS, 28, 2)
        cv2.putText(frame, "TARGET", (cx + 16, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)

    # Info panel (black box, top-left)
    rows = 8 if angles else 4
    panel_h = 22 + rows * 22
    cv2.rectangle(frame, (0, 0), (270, panel_h), (0, 0, 0), -1)
    cv2.rectangle(frame, (0, 0), (270, panel_h), (60, 60, 60), 1)

    def txt(text: str, row: int, colour: tuple = (220, 220, 220)) -> None:
        cv2.putText(frame, text, (8, 20 + row * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, colour, 1, cv2.LINE_AA)

    status_col = ((0, 140, 255) if is_moving
                  else (0, 220, 100) if angles
                  else (80, 80, 255))
    txt(f"Status : {status}", 0, status_col)

    if robot_pos:
        rx, ry, rz = robot_pos
        txt(f"Target : {rx:.1f}, {ry:.1f}, {rz:.1f} cm", 1)
    else:
        txt("Target : —", 1)

    if angles:
        txt(f"Base   : {angles['base']:.0f}\u00b0",       2)
        txt(f"Shldr  : {angles['shoulder']:.0f}\u00b0",   3)
        txt(f"Elbow  : {angles['elbow']:.0f}\u00b0",      4)
        txt(f"L.Wrist: {angles['lower_wrist']:.0f}\u00b0", 5)
        txt(f"Wrist  : {angles['wrist']:.0f}\u00b0",      6)

    hint_row = 8 if angles else 3
    txt("Click=aim  G=grab  H=home  Q=quit", hint_row, (160, 160, 80))


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
            "Try changing CAMERA_INDEX to 1 or 2."
        )

    WINDOW = "Robotic Arm — Click to Aim"
    cv2.namedWindow(WINDOW)

    # Shared mutable state (main thread writes click; worker thread reads it)
    app: dict = {
        "click":     None,   # (cx, cy) pixel of last click
        "new_click": False,  # True when a click hasn't been processed yet
        "angles":    None,   # last computed IK solution
        "robot_pos": None,   # last computed world position
        "status":    "Click on the target object",
        "is_moving": False,
    }

    def on_mouse(event: int, x: int, y: int, flags: int, _param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            app["click"]     = (x, y)
            app["new_click"] = True

    cv2.setMouseCallback(WINDOW, on_mouse)

    print("[VIS] Camera ready.")
    print("      Left-click on target in the video → arm aims there")
    print("      G = grab sequence   H = home   Q = quit")

    # ── Worker: runs servo movement without blocking the video loop ───────────
    def do_move(target_angles: dict) -> None:
        app["is_moving"] = True
        app["status"]    = "Moving…"
        smooth_move(target_angles)
        app["is_moving"] = False
        app["status"]    = "Ready — click a new target or press G"

    def do_grab(target_angles: dict) -> None:
        app["is_moving"] = True
        app["status"]    = "Grabbing…"
        _set_servo_raw("claw", LIMITS["claw"][0])    # open
        time.sleep(0.3)
        smooth_move(target_angles)
        time.sleep(HOLD_SECONDS)
        _set_servo_raw("claw", LIMITS["claw"][1])    # close
        time.sleep(0.6)
        time.sleep(HOLD_SECONDS)
        _set_servo_raw("claw", LIMITS["claw"][0])    # release
        time.sleep(0.3)
        move_home()
        app["is_moving"] = False
        app["status"]    = "Ready — click on target"

    def do_home() -> None:
        app["is_moving"] = True
        app["status"]    = "Homing…"
        move_home()
        app["is_moving"] = False
        app["angles"]    = None
        app["status"]    = "Click on the target object"

    # ── Main video + control loop ─────────────────────────────────────────────
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            # Process new click (only when arm is free)
            if app["new_click"] and not app["is_moving"]:
                app["new_click"] = False
                cx, cy = app["click"]

                pos = pixel_to_robot_frame(cx, cy)
                if pos is None:
                    app["status"]    = "Ray missed table — click lower"
                    app["angles"]    = None
                    app["robot_pos"] = None
                else:
                    angles = ik_solve(*pos)
                    app["robot_pos"] = pos
                    app["angles"]    = angles
                    if angles:
                        threading.Thread(
                            target=do_move, args=(angles,), daemon=True
                        ).start()
                    else:
                        app["status"] = "Out of reach — click closer to arm"

            draw_hud(frame,
                     app["click"],
                     app["angles"],
                     app["robot_pos"],
                     app["status"],
                     app["is_moving"])
            cv2.imshow(WINDOW, frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            elif key == ord('h') and not app["is_moving"]:
                threading.Thread(target=do_home, daemon=True).start()

            elif key == ord('g'):
                if app["angles"] and not app["is_moving"]:
                    threading.Thread(
                        target=do_grab, args=(app["angles"],), daemon=True
                    ).start()
                elif not app["angles"]:
                    app["status"] = "Click a target first, then press G"

    finally:
        cap.release()
        cv2.destroyAllWindows()
        move_home()
        print("[INFO] Shut down cleanly.")


if __name__ == "__main__":
    main()
