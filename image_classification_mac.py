import tensorflow as tf
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input, decode_predictions
from tensorflow.keras.preprocessing import image
import numpy as np
import sys

# Load pretrained model
model = MobileNetV2(weights='imagenet')

def classify(img_path):
    img = image.load_img(img_path, target_size=(224, 224))
    x = image.img_to_array(img)
    x = np.expand_dims(x, axis=0)
    x = preprocess_input(x)

    preds = model.predict(x)
    decoded = decode_predictions(preds, top=3)[0]

    for i, (imagenet_id, label, confidence) in enumerate(decoded):
        print(f"{i+1}. {label}: {confidence * 100:.2f}%")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python classify_image.py <path_to_image>")
    else:
        classify(sys.argv[1])
