import cv2

camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if not camera.isOpened():
    print("Could not open camera")
    exit()

print("Camera opened successfully")

while True:

    ret, frame = camera.read()

    if not ret:
        print("Could not read frame")
        break

    cv2.imshow("Camera", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()