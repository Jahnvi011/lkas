# Camera-Based Lane Keeping Assist (LKAS) Demo

A computer-vision lane keeping demo written in Python with OpenCV. It reads frames from a webcam, detects the left and right lane lines, estimates how far the vehicle is from the lane center, and computes a steering angle in degrees. The angle can optionally be sent to an ESP32 over a serial connection.

> **Safety notice:** This is an educational / prototype project. It only produces a steering angle request. It does **not** control speed or brakes, and it is **not** suitable for use on real roads or in real vehicles. Test only on a small-scale model or bench setup.

---

## Table of Contents

1. [Features](#features)
2. [How It Works (Workflow)](#how-it-works-workflow)
3. [Steering Calculation](#steering-calculation)
4. [Requirements](#requirements)
5. [Installation](#installation)
6. [Usage](#usage)
7. [Configuration](#configuration)
8. [ESP32 Serial Protocol](#esp32-serial-protocol)
9. [Example Output](#example-output)
10. [On-Screen Display](#on-screen-display)
11. [Known Limitations](#known-limitations)
12. [Troubleshooting](#troubleshooting)
13. [Project Structure](#project-structure)

---

## Features

- Real-time lane detection from a standard webcam
- Canny edge detection with a trapezoidal region of interest (ROI)
- Hough line detection with left/right classification by slope
- Proportional (P) steering controller with exponential smoothing
- Output clamped to a safe maximum steering angle (±30° by default)
- Optional serial output to an ESP32 (one signed angle per line)
- Automatic fallback to camera-only mode if the ESP32 is unavailable
- Neutral steering (0°) requested when the lane is lost

---

## How It Works (Workflow)

Each camera frame goes through the following pipeline:

```text
Camera frame
    │
    ▼
1. Grayscale conversion
    │
    ▼
2. Canny edge detection (thresholds 50 / 150)
    │
    ▼
3. Region-of-interest mask (trapezoid in lower half of image)
    │
    ▼
4. Probabilistic Hough transform  →  short line segments
    │
    ▼
5. Segment filtering and classification
      • skip vertical segments (avoid divide-by-zero)
      • skip near-horizontal segments (|slope| < 0.4)
      • negative slope → left lane,  positive slope → right lane
    │
    ▼
6. Line fitting (np.polyfit, y = slope·x + intercept) for each side
    │
    ▼
7. Evaluate both lines at y = 80% of image height
      → left_x, right_x
    │
    ▼
8. Validity check
      • both x values inside the image
      • left_x < right_x
    │
    ▼
9. Lane center vs. vehicle center  →  raw error (pixels)
    │
    ▼
10. Exponential smoothing  →  smoothed error
    │
    ▼
11. P-controller + clamp  →  steering angle (degrees)
    │
    ▼
12. Display overlay  +  optional serial write to ESP32
```

### Step details

| Step | Description |
|---|---|
| **ROI mask** | Keeps only a trapezoid with corners at the bottom-left, bottom-right, and at 35% / 65% of the width at 50% of the height. Edges outside the road area are discarded. |
| **Hough transform** | `HoughLinesP` with `threshold=40`, `minLineLength=40`, `maxLineGap=30` finds short straight segments in the masked edge image. |
| **Left/right split** | In image coordinates (y increases downward), left lane markings have a negative slope and right markings have a positive slope. |
| **Lane fit** | All endpoints on each side are combined and fitted with a first-order polynomial. |
| **Sample row** | Lane positions are evaluated at `y = 0.80 × image height`, a row near the front of the vehicle. |
| **Lost lane** | If no valid lane is found, the smoothing filter resets to 0 and a 0° command is requested. |

---

## Steering Calculation

```text
raw_error       = lane_center_x − vehicle_center_x            (pixels)
smoothed_error  = ALPHA · raw_error + (1 − ALPHA) · previous  (pixels)
steering_angle  = clip(Kp · smoothed_error, −MAX, +MAX)       (degrees)
```

- `vehicle_center_x` is assumed to be the center of the image (`width / 2`), so the camera must be mounted on the vehicle centerline and pointing straight ahead.
- `Kp` has units of **degrees per pixel**.
- `ALPHA` closer to 0 gives heavier smoothing (slower response); closer to 1 uses mostly the new measurement.
- The value is a **requested angle relative to straight ahead**. The ESP32 firmware must convert it into the servo's calibrated position.

---

## Requirements

- Python 3.8 or newer
- A USB webcam
- Python packages:
  - `opencv-python`
  - `numpy`
  - `pyserial` (only if using the ESP32 output)
- *(Optional)* An ESP32 board with firmware that reads angle commands over serial

> **Windows note:** the script opens the camera with `cv2.CAP_DSHOW`, which is a Windows (DirectShow) backend. On Linux or macOS, change the line to `cv2.VideoCapture(0)`.

---

## Installation

```bash
# 1. Clone the repository
git clone https://github.com/<your-username>/<your-repo>.git
cd <your-repo>

# 2. (Recommended) create a virtual environment
python -m venv venv
# Windows
venv\Scripts\activate
# Linux / macOS
source venv/bin/activate

# 3. Install dependencies
pip install opencv-python numpy pyserial
```

You can also create a `requirements.txt`:

```text
opencv-python
numpy
pyserial
```

and install with `pip install -r requirements.txt`.

---

## Usage

### Camera-only mode (default)

```bash
python lkas.py
```

A window titled **LKAS Camera** opens showing the live feed with lane markers and error values. Steering angles are printed to the console.

Press **`q`** or **`Esc`** in the camera window to stop the program.

### With ESP32 output

1. Edit the settings at the top of the script:

   ```python
   USE_ESP32 = True
   ESP32_PORT = "COM5"      # e.g. "/dev/ttyUSB0" on Linux
   BAUD_RATE = 115200
   ```

2. Connect the ESP32 and run:

   ```bash
   python lkas.py
   ```

If the serial port cannot be opened, the script prints an error and continues in camera-only mode. If a write fails during the run, serial output is disabled and the program keeps running.

---

## Configuration

All tunable settings are near the top of the script.

| Parameter | Default | Description |
|---|---|---|
| `Kp_DEG_PER_PIXEL` | `0.5` | Controller gain in degrees per pixel of lane-center error |
| `MAX_STEERING_DEG` | `30.0` | Maximum requested steering angle in either direction |
| `ALPHA` | `0.2` | Error smoothing factor (0–1) |
| `USE_ESP32` | `False` | Enable serial output |
| `ESP32_PORT` | `"COM5"` | Serial port name |
| `BAUD_RATE` | `115200` | Serial baud rate |

Vision parameters inside the main loop (tune for your camera and track):

| Parameter | Location | Effect |
|---|---|---|
| Canny thresholds `50, 150` | `cv2.Canny` | Edge sensitivity |
| ROI polygon | `polygon` | Region where lanes are searched |
| Hough `threshold`, `minLineLength`, `maxLineGap` | `cv2.HoughLinesP` | Line segment sensitivity |
| Slope cutoff `0.4` | slope filter | Ignore near-horizontal segments |
| `y_position = height * 0.80` | lane fit | Row used to measure lane position |

---

## ESP32 Serial Protocol

- **Direction:** PC → ESP32
- **Settings:** 115200 baud, 8N1 (default `pyserial` settings)
- **Format:** one signed decimal angle in degrees per line, ASCII, terminated by `\n`
- **Update rate:** once per processed camera frame
- **Lost lane:** `0.00` is sent

Example stream:

```text
-3.80
-7.74
-11.84
0.00
```

### What the ESP32 firmware needs to do

- Read a line from `Serial` and parse it as a float
- Clamp the value to the mechanical limits of your steering linkage
- Convert degrees to the servo's calibrated pulse width / position
- *(Recommended)* Return to neutral if no command arrives for a set time (watchdog / timeout)

The ESP32 firmware is not included in this repository. *(Add a link or a `/firmware` folder here if you publish it.)*

---

## Example Output

Typical console output in camera-only mode:

```text
Camera-only mode: ESP32 output disabled.
Camera opened successfully.
Press q or Esc in the camera window to stop the program.
Steering: -3.80 deg
Steering: -7.74 deg
Steering: -11.84 deg
Steering: -18.32 deg
Steering: -30.00 deg
Invalid lane detection
Steering: +0.50 deg
LKAS program stopped.
```

Full sample log: see [`lkas_output.md`](lkas_output.md).

### Status messages

| Message | Meaning |
|---|---|
| `Steering: ±X.XX deg` | A valid lane was found and an angle was computed |
| `Lane not detected` | No left or right lane segments found |
| `Invalid lane detection` | Fitted lines fell outside the image, or left was not to the left of right |
| `Invalid lane: near-horizontal fit` | A fitted line had a near-zero slope |
| `Lane fit failed: ...` | `np.polyfit` raised an error |

---

## On-Screen Display

| Overlay | Meaning |
|---|---|
| Green dot | Left lane position at the sample row |
| Red dot | Right lane position at the sample row |
| Yellow dot | Lane center |
| Blue line | Vehicle center (image center) |
| Text (top-left) | Raw error, smoothed error, steering status |
| Text (bottom-left) | `ESP32 ENABLED` or `CAMERA ONLY` |

---

## Known Limitations

- Assumes the camera is centered and level; a mounting offset causes a constant steering bias.
- Straight-line model only; sharp curves are not handled well.
- Sensitive to lighting, shadows, glare, and faded or missing lane markings.
- Steering saturates at ±30° when the error is large (e.g. error above 60 px with the default gain), which can cause oscillation.
- Pixel error is not converted to real-world distance (no camera calibration or perspective transform).
- The controller is proportional only (no derivative or integral action, no speed dependence).
- Frame rate depends on the PC and camera; there is no fixed control loop timing.
- No built-in fail-safe on the ESP32 side if the PC stops sending data.

---

## Troubleshooting

| Problem | Possible fix |
|---|---|
| `Could not open camera` | Check the camera index (`0`, `1`, ...), close other apps using the camera, or remove `cv2.CAP_DSHOW` on non-Windows systems |
| `Could not connect to ESP32` | Verify the port name, cable, drivers, and that no other program (e.g. Serial Monitor) holds the port |
| Lanes never detected | Adjust the ROI polygon, Canny thresholds, or Hough parameters; improve lighting |
| Steering jitters | Lower `ALPHA` or lower `Kp_DEG_PER_PIXEL` |
| Steering reacts too slowly | Raise `ALPHA` or `Kp_DEG_PER_PIXEL` |
| Steering is always biased to one side | Re-center the camera, or apply an offset to `vehicle_center` |
| Frequent `Invalid lane detection` | Tighten the ROI and slope filter to reject noise from other edges |

---

## Project Structure

```text
.
├── lkas.py          # Main program (rename to match your file)
├── lkas_output.md   # Sample console output
├── requirements.txt # Python dependencies (optional)
└── README.md        # This file
```
