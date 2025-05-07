import cv2
import numpy as np
import tensorflow as tf
import json

#classify model
classifier_model_path = "/home/witasme/Ball_Sorting/code/trained_classifier.h5"  #update on your own pi
label_path = "/home/witasme/Ball_Sorting/code/label_map.json"
image_path = "/home/witasme/Ball_Sorting/dataset/test/image.jpg"
img_height = 224
img_width = 224

#load model and label
classifier_model = tf.keras.models.load_model(classifier_model_path)
with open(label_path, 'r') as f:
    label_map = json.load(f)
    labels = {v: k for k, v in label_map.items()}

# load frozen feature extractor
base_model = tf.keras.applications.MobileNetV2(input_shape=(img_height, img_width, 3), include_top=False, weights='imagenet')
base_model.trainable = False
inputs = tf.keras.layers.Input(shape=(img_height, img_width, 3))
x = tf.keras.layers.Rescaling(1. / 255)(inputs)
x = base_model(x, training=False)
feature_extractor = tf.keras.models.Model(inputs, tf.keras.layers.GlobalAveragePooling2D()(x))

#Preprocess image
def preprocess_image(img_path):
    img = tf.keras.utils.load_img(img_path, target_size=(img_height, img_width))
    img_array = tf.keras.utils.img_to_array(img)
    normalized_img = np.expand_dims(img_array, axis=0) / 255.0
    return normalized_img

#predict preprocessed image
def predict_ball(image_path):
    processed_image = preprocess_image(image_path)

    # Extract features using the frozen base model
    features = feature_extractor.predict(processed_image)

    # Make prediction using the trained classifier
    predictions = classifier_model.predict(features)[0]
    predicted_index = np.argmax(predictions)
    confidence = predictions[predicted_index]
    predicted_label = labels.get(predicted_index, "Unknown")

    print(f"Predicted: {predicted_label} (Confidence: {confidence:.2f})")
    return predicted_label, confidence

#make prediction
predicted_label, confidence = predict_ball(image_path)