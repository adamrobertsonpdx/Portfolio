import tensorflow as tf
from tensorflow.keras.preprocessing import image_dataset_from_directory
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.layers import Input, GlobalAveragePooling2D
from tensorflow.keras.models import Model
import numpy as np
import os
import json

#configuration
#set up your photos as follows:
#datset
#   train
#       type 1
#       type 2 etc
#   validation
#       type 1
#       type 2 etc
train_dir = "/home/witasme/Ball_Sorting/dataset/train"
val_dir = "/home/witasme/Ball_Sorting/dataset/validation"
img_size = (224, 224) #sets image sizes
batch_size = 8  # Reduced batch size for extraction
output_dir = "/home/witasme/Ball_Sorting/features" # Define the output directory

# Create the output directory if it doesn't exist
os.makedirs(output_dir, exist_ok=True)

#loads raw image datasets
train_ds_raw = image_dataset_from_directory(
    train_dir, shuffle=False, batch_size=batch_size, image_size=img_size
)

val_ds_raw = image_dataset_from_directory(
    val_dir, shuffle=False, batch_size=batch_size, image_size=img_size
)

#get class names and set them as json map
class_names = train_ds_raw.class_names
num_classes = len(class_names)
label_map = {name: idx for idx, name in enumerate(class_names)}
with open(os.path.join(output_dir, "label_map.json"), "w") as f:
    json.dump(label_map, f)

#feature extraction for the base model (MobilenetV2)
base_model = MobileNetV2(input_shape=img_size + (3,), include_top=False, weights='imagenet')
base_model.trainable = False  # Freeze the base model

#feature extraction model
inputs = Input(shape=img_size + (3,))
x = tf.keras.layers.Rescaling(1. / 255)(inputs)
x = base_model(x, training=False)
feature_extractor = Model(inputs, GlobalAveragePooling2D()(x))

#extract and save features in chunks
def extract_and_save_features(dataset, filename_prefix):
    for i, (images, labels) in enumerate(dataset):
        features = feature_extractor.predict(images)
        features_filename = os.path.join(output_dir, f"{filename_prefix}_features_chunk_{i}.npy")
        labels_filename = os.path.join(output_dir, f"{filename_prefix}_labels_chunk_{i}.npy")
        np.save(features_filename, features)
        np.save(labels_filename, labels.numpy())
        print(f"Processed and saved chunk {i} with {features.shape[0]} images.")

print("Extracting and saving training features in chunks...")
extract_and_save_features(train_ds_raw, "train")

print("Extracting and saving validation features in chunks...")
extract_and_save_features(val_ds_raw, "val")

print("Feature extraction complete. Run the classifier training script next.")