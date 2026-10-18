import cv2
import numpy as np
import time
import serial

# -----------------------------
# P CONTROLLER SETTINGS
# -----------------------------

Kp = 0.5
Ki = 0.0
Kd = 0.0

# Separate variable for smoothing
smoothed_error = 0


# -----------------------------
# SERIAL COMMUNICATION
# -----------------------------

# Change COM5 later to your ESP32's actual COM port
esp32 = serial.Serial(
    "COM5",
    115200,
    timeout=0.1
)

time.sleep(2)

print("Serial connection ready")


# -----------------------------
# OPEN CAMERA
# -----------------------------

camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if not camera.isOpened():
    print("Could not open camera")
    exit()

print("Camera opened successfully")


# -----------------------------
# MAIN LOOP
# -----------------------------

while True:

    # -----------------------------
    # READ CAMERA FRAME
    # -----------------------------

    ret, img = camera.read()

    if not ret:
        print("Could not read frame")
        break

    height, width = img.shape[:2]


    # -----------------------------
    # GRAYSCALE
    # -----------------------------

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    # -----------------------------
    # CANNY EDGES
    # -----------------------------

    edges = cv2.Canny(gray, 50, 150)


    # -----------------------------
    # REGION OF INTEREST
    # -----------------------------

    mask = np.zeros_like(edges)

    polygon = np.array([[
        (0, height),
        (width, height),
        (int(width * 0.65), int(height * 0.50)),
        (int(width * 0.35), int(height * 0.50))
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)

    roi_edges = cv2.bitwise_and(edges, mask)


    # -----------------------------
    # HOUGH LINES
    # -----------------------------

    lines = cv2.HoughLinesP(
        roi_edges,
        rho=1,
        theta=np.pi / 180,
        threshold=40,
        minLineLength=40,
        maxLineGap=30
    )


    # -----------------------------
    # STORE LEFT AND RIGHT LINES
    # -----------------------------

    left_points = []
    right_points = []


    # -----------------------------
    # SEPARATE LANES
    # -----------------------------

    if lines is not None:

        for line in lines:

            x1, y1, x2, y2 = line.reshape(4)

            # Ignore vertical lines
            if x2 == x1:
                continue

            slope = (y2 - y1) / (x2 - x1)

            # Ignore nearly horizontal lines
            if abs(slope) < 0.4:
                continue

            # Left lane
            if slope < 0:

                left_points.append((x1, y1))
                left_points.append((x2, y2))

            # Right lane
            elif slope > 0:

                right_points.append((x1, y1))
                right_points.append((x2, y2))


    # -----------------------------
    # LANE DETECTION
    # -----------------------------

    if len(left_points) > 0 and len(right_points) > 0:

        left_points = np.array(left_points)
        right_points = np.array(right_points)


        # -----------------------------
        # FIT LEFT LANE
        # -----------------------------

        left_slope, left_intercept = np.polyfit(
            left_points[:, 0],
            left_points[:, 1],
            1
        )


        # -----------------------------
        # FIT RIGHT LANE
        # -----------------------------

        right_slope, right_intercept = np.polyfit(
            right_points[:, 0],
            right_points[:, 1],
            1
        )


        # -----------------------------
        # LOOK-AHEAD POSITION
        # -----------------------------

        y_position = int(height * 0.80)


        # -----------------------------
        # CALCULATE LANE POSITIONS
        # -----------------------------

        left_x = int(
            (y_position - left_intercept) / left_slope
        )

        right_x = int(
            (y_position - right_intercept) / right_slope
        )


        # -----------------------------
        # REMOVE IMPOSSIBLE DETECTIONS
        # -----------------------------

        if left_x < 0 or left_x > width:
            continue

        if right_x < 0 or right_x > width:
            continue


        # -----------------------------
        # LANE CENTER
        # -----------------------------

        lane_center = (left_x + right_x) / 2


        # -----------------------------
        # VEHICLE / CAMERA CENTER
        # -----------------------------

        vehicle_center = width / 2


        # -----------------------------
        # RAW LANE ERROR
        # -----------------------------

        raw_error = lane_center - vehicle_center


        # -----------------------------
        # SMOOTH LANE ERROR
        # -----------------------------

        alpha = 0.2

        smoothed_error = (alpha * raw_error) + ((1 - alpha) * smoothed_error)


        # -----------------------------
        # P CONTROLLER
        # -----------------------------

        P = Kp * smoothed_error

        steering_command = P


        # -----------------------------
        # LIMIT STEERING COMMAND
        # -----------------------------

        steering_command = max(
            -30,
            min(30, steering_command)
        )


        # -----------------------------
        # PRINT VALUES
        # -----------------------------

        print(
            f"Lane Center: {lane_center:.1f} | "
            f"Vehicle Center: {vehicle_center:.1f} | "
            f"Raw Error: {raw_error:.1f}"
        )

        print(
            f"Smooth Error: {smoothed_error:.1f} | "
            f"ESP32 COMMAND: {steering_command:.2f}"
        )


        # -----------------------------
        # SEND TO ESP32 LATER
        # -----------------------------

        # esp32.write(
        #     f"{steering_command:.2f}\n".encode()
        # )


        # -----------------------------
        # DRAW LEFT LANE POINT
        # -----------------------------

        cv2.circle(
            img,
            (left_x, y_position),
            8,
            (0, 255, 0),
            -1
        )


        # -----------------------------
        # DRAW RIGHT LANE POINT
        # -----------------------------

        cv2.circle(
            img,
            (right_x, y_position),
            8,
            (0, 0, 255),
            -1
        )


        # -----------------------------
        # DRAW LANE CENTER
        # -----------------------------

        cv2.circle(
            img,
            (int(lane_center), y_position),
            8,
            (0, 255, 255),
            -1
        )


        # -----------------------------
        # DRAW VEHICLE CENTER
        # -----------------------------

        cv2.line(
            img,
            (int(vehicle_center), y_position),
            (int(vehicle_center), height),
            (255, 0, 0),
            3
        )


        # -----------------------------
        # DISPLAY RAW ERROR
        # -----------------------------

        cv2.putText(
            img,
            f"Raw Error: {raw_error:.2f}",
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )


        # -----------------------------
        # DISPLAY SMOOTH ERROR
        # -----------------------------

        cv2.putText(
            img,
            f"Smooth Error: {smoothed_error:.2f}",
            (30, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )


        # -----------------------------
        # DISPLAY STEERING
        # -----------------------------

        cv2.putText(
            img,
            f"Steering: {steering_command:.2f}",
            (30, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )


    else:

        # -----------------------------
        # NO LANES DETECTED
        # -----------------------------

        cv2.putText(
            img,
            "Lane not detected",
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )


    # -----------------------------
    # DISPLAY CAMERA
    # -----------------------------

    cv2.imshow("LKAS Camera", img)


    # -----------------------------
    # PRESS Q TO QUIT
    # -----------------------------

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# -----------------------------
# CLEAN UP
# -----------------------------

camera.release()

cv2.destroyAllWindows()

esp32.close()