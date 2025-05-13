import cv2
from ultralytics import YOLO

# Load YOLO model
model = YOLO("last300E.pt")
model.to('cuda')
# Open the video (1 = webcam, or replace with 'video.mp4')
input_video = 1
cap = cv2.VideoCapture(input_video)

# Get video properties
fps = cap.get(cv2.CAP_PROP_FPS)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Set up video writer
output_video = 'output13.mp4'
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

# Confidence threshold
CONFIDENCE_THRESHOLD = 0.75

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO on the frame
    results = model(frame)  

    # Filter out low-confidence detections
    result = results[0]
    high_conf_boxes = result.boxes[result.boxes.conf > CONFIDENCE_THRESHOLD]  # Keep only confident predictions

    # Replace original boxes with filtered ones
    result.boxes = high_conf_boxes  

    # Apply YOLO's original `.plot()` method
    annotated_frame = result.plot()

    # Write the annotated frame to output video
    out.write(annotated_frame)
    cv2.imshow("Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
