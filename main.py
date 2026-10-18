import cv2
import numpy as np

# 1. Load image
img = cv2.imread(
    r"C:\Users\Jahnv\Downloads\roadforlkas.webp"
)

if img is None:
    print("Could not find roadforlkas.webp")
    exit()

height, width = img.shape[:2]

print("Image width:", width)
print("Image height:", height)

# 2. Grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 3. Edges
edges = cv2.Canny(gray, 50, 150)

# 4. ROI
mask = np.zeros_like(edges)

polygon = np.array([[
    (0, height),
    (width, height),
    (int(width * 0.65), int(height * 0.50)),
    (int(width * 0.35), int(height * 0.50))
]], np.int32)

cv2.fillPoly(mask, polygon, 255)

roi_edges = cv2.bitwise_and(edges, mask)

# 5. Hough lines
lines = cv2.HoughLinesP(
    roi_edges,
    rho=1,
    theta=np.pi / 180,
    threshold=40,
    minLineLength=40,
    maxLineGap=30
)

left_points = []
right_points = []

# 6. Separate lines
if lines is not None:

    for line in lines:

        x1, y1, x2, y2 = line.reshape(4)

        if x2 == x1:
            continue

        slope = (y2 - y1) / (x2 - x1)

        if abs(slope) < 0.4:
            continue

        if slope < 0:
            left_points.append((x1, y1))
            left_points.append((x2, y2))

        elif slope > 0:
            right_points.append((x1, y1))
            right_points.append((x2, y2))


print("Left points:", len(left_points))
print("Right points:", len(right_points))


# 7. Fit left lane
if len(left_points) > 0:

    left_points = np.array(left_points)

    left_slope, left_intercept = np.polyfit(
        left_points[:, 0],
        left_points[:, 1],
        1
    )

    print("Left lane fitted")


# 8. Fit right lane
if len(right_points) > 0:

    right_points = np.array(right_points)

    right_slope, right_intercept = np.polyfit(
        right_points[:, 0],
        right_points[:, 1],
        1
    )

    print("Right lane fitted")

# 9. Calculate lane positions at a look-ahead point
y_bottom = int(height * 0.80)

if len(left_points) > 0 and len(right_points) > 0:

    left_x = int(
        (y_bottom - left_intercept) / left_slope
    )

    right_x = int(
        (y_bottom - right_intercept) / right_slope
    )

    # 10. Lane center
    lane_center = (left_x + right_x) / 2

    # 11. Vehicle/camera center
    vehicle_center = width / 2

    # 12. Lane error
    lane_error = lane_center - vehicle_center

    print()
    print("LEFT LANE X:", left_x)
    print("RIGHT LANE X:", right_x)
    print("LANE CENTER:", lane_center)
    print("VEHICLE CENTER:", vehicle_center)
    print("LANE ERROR:", lane_error)
# PID controller

Kp = 0.5
Ki = 0.05
Kd = 0.1

previous_error = 0
integral = 0

dt = 0.1

# Simulated lane errors from consecutive frames
lane_errors = [-12.5, -11.0, -8.0, -5.0, -2.0, 0.0, 2.0]

for lane_error in lane_errors:

    # Proportional term
    P = Kp * lane_error

    # Integral term
    integral += lane_error * dt
    I = Ki * integral

    # Derivative term
    D = Kd * (lane_error - previous_error) / dt

    # PID output
    steering_command = P + I + D

    # Limit steering command
    if steering_command > 30:
        steering_command = 30

    if steering_command < -30:
        steering_command = -30

    print()
    print("Lane Error:", lane_error)
    print("P:", P)
    print("I:", I)
    print("D:", D)
    print("Steering Command:", steering_command)

    # Store error for next frame
    previous_error = lane_error
# 13. Display
result = img.copy()

if len(left_points) > 0 and len(right_points) > 0:

    cv2.circle(
        result,
        (left_x, y_bottom),
        10,
        (0, 255, 0),
        -1
    )

    cv2.circle(
        result,
        (right_x, y_bottom),
        10,
        (0, 0, 255),
        -1
    )

    cv2.line(
        result,
        (int(lane_center), y_bottom),
        (int(lane_center), int(height * 0.5)),
        (0, 255, 255),
        4
    )

    cv2.line(
        result,
        (int(vehicle_center), y_bottom),
        (int(vehicle_center), int(height * 0.5)),
        (255, 0, 0),
        4
    )

cv2.imshow("Road", result)
cv2.imshow("ROI Edges", roi_edges)

cv2.waitKey(0)
cv2.destroyAllWindows()