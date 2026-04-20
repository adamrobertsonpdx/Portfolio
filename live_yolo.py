from ultralytics import YOLO
import cv2

# Load YOLOv8m pretrained model
model = YOLO("yolov8x.pt")

# Choose video source
# 0 for USB webcam or use 'http://<pi_ip>:8080/?action=stream'
video_source = 0
cap = cv2.VideoCapture(video_source)

if not cap.isOpened():
    print("Error: Couldn't open video source.")
    exit()

label = "..."

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLOv8 prediction
    results = model.predict(source=frame, verbose=False)

    # Extract top label
    if results and results[0].boxes:
        top = results[0].names[int(results[0].boxes.cls[0])]
        conf = float(results[0].boxes.conf[0])
        label = f"{top}: {conf * 100:.1f}%"
    else:
        label = "No object detected"

    # Display label on frame
    cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow("YOLOv8 Classification", frame)

    # Press q to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
