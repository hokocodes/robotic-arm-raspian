#!/usr/bin/env python3
"""
Click-to-target robotic arm controller (WEB version)
=====================================================
The camera is FIXED to the LEFT of the robot arm, facing the workspace.
Control everything from a web browser — no desktop GUI needed (works great
over SSH, and you can control the arm from your laptop or phone too).

How to use
----------
1. Run:  python arm_vision_control.py
2. Open the URL it prints, e.g.  http://<your-pi-ip>:5000
3. CLICK anywhere on the live video to aim the arm at that spot.
   The arm moves there immediately.
4. Use the on-screen buttons:
     Grab  — open claw → move to target → close → retreat home
     Home  — return arm to home position

Why a web app?
--------------
Raspberry Pi OpenCV is often built without desktop-window (GUI) support, so
cv2.imshow() fails with "The function is not implemented".  Streaming the video
to a browser sidesteps that completely — cv2 is only used to capture and encode
frames, never to open a window.

Depth is computed by casting a ray from the camera through the clicked pixel and
finding where it hits the table surface (z = TARGET_Z_CM).  No colour detection
or object-size calibration needed.

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
from flask import Flask, Response, jsonify, request

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
CAM_PAN_DEG  = -90.0   # do NOT change unless your camera faces a different way
CAM_TILT_DEG =  15.0   # degrees tilted downward (0 = perfectly level)

# ── Target height ─────────────────────────────────────────────────────────────
# The script intersects the camera ray with a horizontal plane at this height.
#   0.0 = aim at the table surface itself
#   3.0 = aim 3 cm above the table (top of a small object)
TARGET_Z_CM = 3.0

# ── Web server ────────────────────────────────────────────────────────────────
WEB_HOST = "0.0.0.0"   # 0.0.0.0 = reachable from other devices on your network
WEB_PORT = 5000

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
    base_deg = math.degrees(math.atan2(y, x)) + 90.0

    reach = math.sqrt(x ** 2 + y ** 2)
    r     = math.sqrt(reach ** 2 + z ** 2)

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

    wrist_deg = (90.0
                 + math.degrees(shoulder_rad)
                 + (math.degrees(elbow_rad) - 180.0))
    wrist_deg = max(LIMITS["wrist"][0], min(LIMITS["wrist"][1], wrist_deg))

    angles = {
        "base":        base_deg,
        "shoulder":    shoulder_deg,
        "elbow":       elbow_deg,
        "lower_wrist": 90.0,
        "wrist":       wrist_deg,
    }

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

    Returns (x, y, z) in robot frame (cm), or None if ray misses the plane.
    """
    f = _focal_px()

    px_x = (cx - FRAME_W / 2.0) * (-1.0 if CAM_FLIP_IMAGE_X else 1.0)
    px_y = -(cy - FRAME_H / 2.0)  # flip: image y grows down, camera y grows up

    # Ray direction in camera frame (x=right, y=up, z=forward)
    d = np.array([px_x / f, px_y / f, 1.0])

    # Tilt (rotate around camera x-axis, positive = nose down)
    tilt = math.radians(CAM_TILT_DEG)
    ct, st = math.cos(tilt), math.sin(tilt)
    d = np.array([d[0],
                  d[1] * ct - d[2] * st,
                  d[1] * st + d[2] * ct])

    # Pan (rotate around world z-axis)
    pan = math.radians(CAM_PAN_DEG)
    cp, sp = math.cos(pan), math.sin(pan)
    d_robot = np.array([
        d[2] * cp - d[0] * sp,
        d[2] * sp + d[0] * cp,
        d[1],
    ])

    origin = np.array([CAM_X, CAM_Y, CAM_Z])

    dz = d_robot[2]
    if abs(dz) < 1e-6:
        return None

    t = (TARGET_Z_CM - origin[2]) / dz
    if t < 0:
        return None

    point = origin + t * d_robot
    return float(point[0]), float(point[1]), float(point[2])


# ═══════════════════════════════════════════════════════════════════════════════
# SHARED STATE  (between camera thread, web requests, and motion threads)
# ═══════════════════════════════════════════════════════════════════════════════

app_state: dict = {
    "click":     None,   # (cx, cy) pixel of last click (in FRAME_W×FRAME_H space)
    "angles":    None,   # last IK solution
    "robot_pos": None,   # last world position (x, y, z) cm
    "status":    "Click on the target object",
    "is_moving": False,
}

_latest_frame: np.ndarray | None = None
_frame_lock = threading.Lock()
_running = True


# ═══════════════════════════════════════════════════════════════════════════════
# CAMERA THREAD  — continuously grabs frames and draws the target marker
# ═══════════════════════════════════════════════════════════════════════════════

def camera_loop() -> None:
    global _latest_frame

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        print(f"[ERROR] Could not open camera index {CAMERA_INDEX}. "
              "Try CAMERA_INDEX = 1 or 2.")
        return

    print("[VIS] Camera streaming started.")

    while _running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        # Draw the clicked target marker on the frame
        if app_state["click"]:
            cx, cy = app_state["click"]
            colour = (0, 140, 255) if app_state["is_moving"] else (0, 220, 100)
            cv2.circle(frame, (cx, cy), 14, colour, 2)
            cv2.drawMarker(frame, (cx, cy), colour, cv2.MARKER_CROSS, 28, 2)

        with _frame_lock:
            _latest_frame = frame

    cap.release()
    print("[VIS] Camera stopped.")


def mjpeg_generator():
    """Yield camera frames as an MJPEG stream for the browser."""
    while _running:
        with _frame_lock:
            frame = None if _latest_frame is None else _latest_frame.copy()
        if frame is None:
            time.sleep(0.03)
            continue
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            continue
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")
        time.sleep(0.03)   # ~30 fps cap


# ═══════════════════════════════════════════════════════════════════════════════
# MOTION WORKERS  (run in background threads so the web server stays responsive)
# ═══════════════════════════════════════════════════════════════════════════════

def do_move(target_angles: dict) -> None:
    app_state["is_moving"] = True
    app_state["status"]    = "Moving…"
    smooth_move(target_angles)
    app_state["is_moving"] = False
    app_state["status"]    = "Ready — click a new target or press Grab"


def do_grab(target_angles: dict) -> None:
    app_state["is_moving"] = True
    app_state["status"]    = "Grabbing…"
    _set_servo_raw("claw", LIMITS["claw"][0])   # open
    time.sleep(0.3)
    smooth_move(target_angles)
    time.sleep(HOLD_SECONDS)
    _set_servo_raw("claw", LIMITS["claw"][1])   # close
    time.sleep(0.6)
    time.sleep(HOLD_SECONDS)
    _set_servo_raw("claw", LIMITS["claw"][0])   # release
    time.sleep(0.3)
    move_home()
    app_state["is_moving"] = False
    app_state["status"]    = "Ready — click on target"


def do_home() -> None:
    app_state["is_moving"] = True
    app_state["status"]    = "Homing…"
    move_home()
    app_state["is_moving"] = False
    app_state["angles"]    = None
    app_state["status"]    = "Click on the target object"


# ═══════════════════════════════════════════════════════════════════════════════
# WEB APP
# ═══════════════════════════════════════════════════════════════════════════════

app = Flask(__name__)

PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robotic Arm — Click to Aim</title>
  <style>
    * { box-sizing: border-box; }
    body { margin:0; background:#0d1117; color:#e6edf3;
           font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; }
    .wrap { max-width: 720px; margin: 0 auto; padding: 16px; }
    h1 { font-size: 1.15rem; font-weight: 600; margin: 8px 0 14px; }
    .feed { position: relative; line-height: 0; border-radius: 12px;
            overflow: hidden; border: 1px solid #30363d; }
    .feed img { width: 100%; height: auto; cursor: crosshair; display:block; }
    .bar { display:flex; gap:10px; margin-top:14px; }
    button { flex:1; padding:14px; font-size:1rem; font-weight:600;
             border:none; border-radius:10px; cursor:pointer; color:#fff; }
    .grab { background:#238636; } .grab:active { background:#1a6e2b; }
    .home { background:#1f6feb; } .home:active { background:#1a5fce; }
    .status { margin-top:14px; padding:12px 14px; background:#161b22;
              border:1px solid #30363d; border-radius:10px; font-size:.95rem; }
    .pos { color:#8b949e; font-size:.85rem; margin-top:6px; }
    .hint { color:#8b949e; font-size:.85rem; margin-top:10px; }
  </style>
</head>
<body>
  <div class="wrap">
    <h1>🦾 Robotic Arm — Click to Aim</h1>
    <div class="feed">
      <img id="feed" src="video_feed" alt="camera feed">
    </div>
    <div class="bar">
      <button class="grab" onclick="send('grab')">Grab</button>
      <button class="home" onclick="send('home')">Home</button>
    </div>
    <div class="status">
      <div id="status">Loading…</div>
      <div class="pos" id="pos"></div>
    </div>
    <div class="hint">Tap or click anywhere on the video to aim the arm there.</div>
  </div>

<script>
const feed = document.getElementById('feed');

feed.addEventListener('click', (e) => {
  const r = feed.getBoundingClientRect();
  const fx = (e.clientX - r.left) / r.width;   // 0..1 across the image
  const fy = (e.clientY - r.top)  / r.height;  // 0..1 down the image
  fetch('click', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({ fx, fy })
  });
});

function send(action) {
  fetch(action, { method: 'POST' });
}

async function poll() {
  try {
    const r = await fetch('status');
    const s = await r.json();
    document.getElementById('status').textContent = 'Status: ' + s.status;
    document.getElementById('pos').textContent = s.pos
      ? `Target: ${s.pos[0].toFixed(1)}, ${s.pos[1].toFixed(1)}, ${s.pos[2].toFixed(1)} cm`
      : '';
  } catch (e) {}
}
setInterval(poll, 500);
poll();
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return PAGE


@app.route("/video_feed")
def video_feed():
    return Response(mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/click", methods=["POST"])
def click():
    if app_state["is_moving"]:
        return jsonify(ok=False, status=app_state["status"])

    data = request.get_json(force=True)
    cx = int(max(0.0, min(1.0, data["fx"])) * FRAME_W)
    cy = int(max(0.0, min(1.0, data["fy"])) * FRAME_H)
    app_state["click"] = (cx, cy)

    pos = pixel_to_robot_frame(cx, cy)
    if pos is None:
        app_state["status"]    = "Ray missed table — click lower"
        app_state["angles"]    = None
        app_state["robot_pos"] = None
        return jsonify(ok=False, status=app_state["status"])

    angles = ik_solve(*pos)
    app_state["robot_pos"] = pos
    app_state["angles"]    = angles

    if angles:
        threading.Thread(target=do_move, args=(angles,), daemon=True).start()
        return jsonify(ok=True, status="Moving…")
    else:
        app_state["status"] = "Out of reach — click closer to arm"
        return jsonify(ok=False, status=app_state["status"])


@app.route("/grab", methods=["POST"])
def grab():
    if app_state["angles"] and not app_state["is_moving"]:
        threading.Thread(target=do_grab,
                         args=(app_state["angles"],), daemon=True).start()
        return jsonify(ok=True)
    if not app_state["angles"]:
        app_state["status"] = "Click a target first, then press Grab"
    return jsonify(ok=False, status=app_state["status"])


@app.route("/home", methods=["POST"])
def home():
    if not app_state["is_moving"]:
        threading.Thread(target=do_home, daemon=True).start()
        return jsonify(ok=True)
    return jsonify(ok=False, status=app_state["status"])


@app.route("/status")
def status():
    return jsonify(status=app_state["status"],
                   pos=app_state["robot_pos"],
                   moving=app_state["is_moving"])


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    move_home()

    # Start camera capture in the background
    threading.Thread(target=camera_loop, daemon=True).start()

    print("\n" + "=" * 56)
    print("  Robotic Arm web control is running!")
    print(f"  On this Pi:        http://localhost:{WEB_PORT}")
    print(f"  From your phone/PC: http://<your-pi-ip>:{WEB_PORT}")
    print("  (find the Pi's IP with:  hostname -I )")
    print("=" * 56 + "\n")

    try:
        app.run(host=WEB_HOST, port=WEB_PORT, threaded=True)
    finally:
        global _running
        _running = False
        time.sleep(0.2)
        move_home()
        print("[INFO] Shut down cleanly.")


if __name__ == "__main__":
    main()
