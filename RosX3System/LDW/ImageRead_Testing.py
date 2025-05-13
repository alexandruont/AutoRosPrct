import cv2

# Open the webcam (0 = default camera, change to 1 or 2 for external cameras)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    ret, frame = cap.read()  # Capture frame-by-frame
    if not ret:
        print("Error: Could not read frame.")
        break

    cv2.imshow("Webcam Feed", frame)  # Display the frame

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close windows
cap.release()
cv2.destroyAllWindows()
