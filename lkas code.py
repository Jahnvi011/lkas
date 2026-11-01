"""Camera-based lane keeping demo with optional degree output to an ESP32.

The steering command is a requested angle relative to straight ahead. The ESP32
must convert that angle to the servo's calibrated position. This script does not
control speed or brakes.

Each frame can also be logged to a CSV file for tuning the controller.
"""

import csv
import time
from datetime import datetime

import cv2
import numpy as np


# -----------------------------
# CONTROLLER SETTINGS
# -----------------------------

# Kp units: degrees per pixel of lane-center error.
Kp_DEG_PER_PIXEL = 0.5
MAX_STEERING_DEG = 30.0

# Error smoothing: 0 means retain the old value; 1 means use only the new value.
ALPHA = 0.2
smoothed_error = 0.0


# -----------------------------
# OPTIONAL ESP32 CONNECTION
# -----------------------------

USE_ESP32 = False
ESP32_PORT = "COM5"
BAUD_RATE = 115200
esp32 = None

if USE_ESP32:
    try:
        import serial

        esp32 = serial.Serial(ESP32_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2.0)  # Some boards reset when the serial port opens.
        print(f"ESP32 connected on {ESP32_PORT}")
    except Exception as exc:
        print(f"Could not connect to ESP32: {exc}")
        print("Continuing in camera-only mode.")
        USE_ESP32 = False
        esp32 = None
else:
    print("Camera-only mode: ESP32 output disabled.")


# -----------------------------
# OPTIONAL CSV LOGGING
# -----------------------------

LOG_TO_CSV = True
FLUSH_EVERY_N_FRAMES = 30  # Limits data loss if the program crashes.
log_file = None
log_writer = None

if LOG_TO_CSV:
    try:
        log_name = datetime.now().strftime("lkas_log_%Y%m%d_%H%M%S.csv")
        log_file = open(log_name, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(
            [
                "time_s",
                "frame",
                "status",
                "left_x",
                "right_x",
                "lane_center",
                "vehicle_center",
                "raw_error_px",
                "smoothed_error_px",
                "steering_deg",
                "sent_deg",
                "n_left_pts",
                "n_right_pts",
                "saturated",
            ]
        )
        print(f"Logging to {log_name}")
    except OSError as exc:
        print(f"Could not open log file: {exc}")
        print("Continuing without CSV logging.")
        LOG_TO_CSV = False
        log_file = None
        log_writer = None

start_time = time.time()
frame_count = 0


# -----------------------------
# OPEN CAMERA
# -----------------------------

camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if not camera.isOpened():
    print("Could not open camera. Check that camera 0 is available.")
    if esp32 is not None and esp32.is_open:
        esp32.close()
    if log_file is not None:
        log_file.close()
    raise SystemExit(1)

print("Camera opened successfully.")
print("Press q or Esc in the camera window to stop the program.")


# -----------------------------
# MAIN LOOP
# -----------------------------

try:
    while True:
        ret, img = camera.read()
        if not ret:
            print("Could not read a camera frame.")
            break

        height, width = img.shape[:2]

        # Reset per-frame values so the log never contains stale data.
        frame_count += 1
        left_x = right_x = lane_center = raw_error = None

        # Convert to grayscale, then find strong image edges.
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Keep edges only in a trapezoid where the road is expected to appear.
        mask = np.zeros_like(edges)
        polygon = np.array(
            [[
                (0, height - 1),
                (width - 1, height - 1),
                (int(width * 0.65), int(height * 0.50)),
                (int(width * 0.35), int(height * 0.50)),
            ]],
            dtype=np.int32,
        )
        cv2.fillPoly(mask, polygon, 255)
        roi_edges = cv2.bitwise_and(edges, mask)

        # Detect short straight line segments inside the road region.
        lines = cv2.HoughLinesP(
            roi_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=40,
            minLineLength=40,
            maxLineGap=30,
        )

        left_points = []
        right_points = []

        # Classify segments by slope sign: negative for left, positive for right.
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)

                if x2 == x1:  # Avoid division by zero for vertical segments.
                    continue

                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < 0.4:  # Ignore nearly horizontal segments.
                    continue

                if slope < 0:
                    left_points.extend(((x1, y1), (x2, y2)))
                else:
                    right_points.extend(((x1, y1), (x2, y2)))

        steering_angle_deg = None
        status = "Lane not detected"

        if left_points and right_points:
            left_points = np.asarray(left_points)
            right_points = np.asarray(right_points)

            try:
                # Fit image-space lines in the form y = slope*x + intercept.
                left_slope, left_intercept = np.polyfit(
                    left_points[:, 0], left_points[:, 1], 1
                )
                right_slope, right_intercept = np.polyfit(
                    right_points[:, 0], right_points[:, 1], 1
                )

                y_position = int(height * 0.80)

                # Avoid dividing by an almost-zero fitted slope.
                if abs(left_slope) < 1e-6 or abs(right_slope) < 1e-6:
                    status = "Invalid lane: near-horizontal fit"
                else:
                    left_x = int((y_position - left_intercept) / left_slope)
                    right_x = int((y_position - right_intercept) / right_slope)

                    valid_lane = (
                        0 <= left_x < width
                        and 0 <= right_x < width
                        and left_x < right_x
                    )

                    if valid_lane:
                        lane_center = (left_x + right_x) / 2.0
                        vehicle_center = width / 2.0
                        raw_error = lane_center - vehicle_center  # pixels

                        smoothed_error = (
                            ALPHA * raw_error + (1.0 - ALPHA) * smoothed_error
                        )

                        # One calculation: pixels * degrees/pixel = degrees.
                        steering_angle_deg = float(
                            np.clip(
                                Kp_DEG_PER_PIXEL * smoothed_error,
                                -MAX_STEERING_DEG,
                                MAX_STEERING_DEG,
                            )
                        )
                        status = f"Steering: {steering_angle_deg:+.2f} deg"

                        cv2.circle(img, (left_x, y_position), 8, (0, 255, 0), -1)
                        cv2.circle(img, (right_x, y_position), 8, (0, 0, 255), -1)
                        cv2.circle(
                            img, (int(lane_center), y_position), 8, (0, 255, 255), -1
                        )
                        cv2.line(
                            img,
                            (int(vehicle_center), y_position),
                            (int(vehicle_center), height - 1),
                            (255, 0, 0),
                            3,
                        )
                        cv2.putText(
                            img,
                            f"Raw error: {raw_error:.1f} px",
                            (30, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (255, 255, 255),
                            2,
                        )
                        cv2.putText(
                            img,
                            f"Smoothed error: {smoothed_error:.1f} px",
                            (30, 75),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (255, 255, 255),
                            2,
                        )
                        cv2.putText(
                            img,
                            status,
                            (30, 110),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (255, 255, 255),
                            2,
                        )
                    else:
                        status = "Invalid lane detection"

            except (ValueError, np.linalg.LinAlgError) as exc:
                status = f"Lane fit failed: {exc}"

        # A lost lane resets the filter and requests neutral steering if serial is on.
        if steering_angle_deg is None:
            smoothed_error = 0.0
            cv2.putText(
                img,
                status,
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            steering_to_send = 0.0
        else:
            steering_to_send = steering_angle_deg

        if USE_ESP32 and esp32 is not None and esp32.is_open:
            try:
                # Protocol: one signed steering angle in degrees per line.
                esp32.write(f"{steering_to_send:.2f}\n".encode("ascii"))
            except Exception as exc:
                print(f"ESP32 write failed: {exc}")
                USE_ESP32 = False
                esp32.close()
                esp32 = None

        print(status)
        mode_text = "ESP32 ENABLED" if USE_ESP32 and esp32 is not None else "CAMERA ONLY"
        cv2.putText(
            img,
            mode_text,
            (30, height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        # Log one row per frame for later tuning and analysis.
        if log_writer is not None:
            saturated = (
                steering_angle_deg is not None
                and abs(steering_angle_deg) >= MAX_STEERING_DEG
            )
            log_writer.writerow(
                [
                    round(time.time() - start_time, 3),
                    frame_count,
                    status,
                    left_x,
                    right_x,
                    None if lane_center is None else round(lane_center, 1),
                    width / 2.0,
                    None if raw_error is None else round(raw_error, 2),
                    round(smoothed_error, 2),
                    None if steering_angle_deg is None else round(steering_angle_deg, 2),
                    round(steering_to_send, 2),
                    len(left_points),
                    len(right_points),
                    int(saturated),
                ]
            )
            if frame_count % FLUSH_EVERY_N_FRAMES == 0:
                log_file.flush()

        cv2.imshow("LKAS Camera", img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:  # q or Esc
            break

finally:
    camera.release()
    cv2.destroyAllWindows()
    if esp32 is not None and esp32.is_open:
        esp32.close()
    if log_file is not None:
        log_file.close()
        print("CSV log saved.")
    print("LKAS program stopped.")