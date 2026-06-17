#!/usr/bin/env python3
"""
HSV Color Tuner
===============
Run this script to find the right HSV range for your target object.

Usage:
    python color_tuner.py

Controls:
  - Adjust the six sliders until ONLY your object appears white in the mask.
  - Press S to save the values — they will be printed for you to copy into
    arm_vision_control.py (COLOR_LOWER1 / COLOR_UPPER1 etc.).
  - Press Q to quit.

Note: Red wraps around 0 and 180 in OpenCV HSV. For red objects the tuner
shows you both ranges so you can fill in both pairs in the main script.
"""

import cv2
import numpy as np

CAMERA_INDEX = 0
FRAME_W = 640
FRAME_H = 480

def nothing(_):
    pass


def main() -> None:
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {CAMERA_INDEX}.")

    win = "HSV Tuner — S=save  Q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    cv2.createTrackbar("H low",  win,   0, 180, nothing)
    cv2.createTrackbar("H high", win, 180, 180, nothing)
    cv2.createTrackbar("S low",  win,   0, 255, nothing)
    cv2.createTrackbar("S high", win, 255, 255, nothing)
    cv2.createTrackbar("V low",  win,   0, 255, nothing)
    cv2.createTrackbar("V high", win, 255, 255, nothing)

    print("Adjust sliders until your object appears white in the right panel.")
    print("Press S to print values, Q to quit.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.resize(frame, (FRAME_W, FRAME_H))
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hl = cv2.getTrackbarPos("H low",  win)
        hh = cv2.getTrackbarPos("H high", win)
        sl = cv2.getTrackbarPos("S low",  win)
        sh = cv2.getTrackbarPos("S high", win)
        vl = cv2.getTrackbarPos("V low",  win)
        vh = cv2.getTrackbarPos("V high", win)

        lower = np.array([hl, sl, vl])
        upper = np.array([hh, sh, vh])

        mask    = cv2.inRange(hsv, lower, upper)
        masked  = cv2.bitwise_and(frame, frame, mask=mask)

        # Side-by-side: original | mask (grey) | masked colour
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        panel    = np.hstack([frame, mask_bgr, masked])

        label = f"H:[{hl}-{hh}]  S:[{sl}-{sh}]  V:[{vl}-{vh}]"
        cv2.putText(panel, label, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        cv2.imshow(win, panel)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            print("\n── Copy these into arm_vision_control.py ──")
            print(f"COLOR_LOWER1 = np.array([{hl}, {sl}, {vl}])")
            print(f"COLOR_UPPER1 = np.array([{hh}, {sh}, {vh}])")
            print("# If your object is RED, also set a second range for hue > 160:")
            print("# COLOR_LOWER2 = np.array([170, S_low, V_low])")
            print("# COLOR_UPPER2 = np.array([180, S_high, V_high])")
            print("───────────────────────────────────────────\n")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
