import cv2
import tensorflow as tf
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input, decode_predictions
import numpy as np
import time

# Load model once
model = tf.keras.applications.MobileNetV2(weights="imagenet")

# Preprocessing helper
def preprocess_frame(frame):
    frame_resized = cv2.resize(frame, (224, 224))
    img_array = np.expand_dims(frame_resized.astype(np.float32), axis=0)
    return preprocess_input(img_array)

# Video capture (0 = default USB cam)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

frame_count = 0
inference_every = 10  # Adjust if you want it faster or slower
label = "..."

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1
    if frame_count % inference_every == 0:
        input_tensor = preprocess_frame(frame)
        preds = model.predict(input_tensor, verbose=0)
        decoded = decode_predictions(preds, top=1)[0]
        label = f"{decoded[0][1]}: {decoded[0][2]*100:.1f}%"

    # Overlay label
    cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                1, (0, 255, 0), 2, cv2.LINE_AA)

    # Display frame
    cv2.imshow("Live Classification", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
