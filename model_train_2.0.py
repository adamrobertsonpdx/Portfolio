import tensorflow as tf
from tensorflow.keras.layers import Input, Dense, Dropout
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam
import numpy as np
import os
import glob

# configuration
feature_vector_size = 1280  # Output size of GlobalAveragePooling2D in MobileNetV2
num_classes = 3
epochs = 50
batch_size = 32
learning_rate = 1e-3
features_dir = "/home/witasme/Ball_Sorting/features" # Update this to your output directory

# defintion of classifier model
inputs = Input(shape=(feature_vector_size,))
x = Dense(256, activation='relu')(inputs)
x = Dropout(0.5)(x)
outputs = Dense(num_classes, activation='softmax')(x)
classifier_model = Model(inputs, outputs)

#compile classifier
classifier_model.compile(optimizer=Adam(learning_rate),
                         loss='sparse_categorical_crossentropy',
                         metrics=['accuracy'])

#load and train on feature chunks
train_feature_files = sorted(glob.glob(os.path.join(features_dir, "train_features_chunk_*.npy")))
train_label_files = sorted(glob.glob(os.path.join(features_dir, "train_labels_chunk_*.npy")))

val_feature_files = sorted(glob.glob(os.path.join(features_dir, "val_features_chunk_*.npy")))
val_label_files = sorted(glob.glob(os.path.join(features_dir, "val_labels_chunk_*.npy")))

print("Training the classifier on feature chunks...")
for epoch in range(epochs):
    print(f"Epoch {epoch+1}/{epochs}")
    for i in range(len(train_feature_files)):
        train_features_chunk = np.load(train_feature_files[i])
        train_labels_chunk = np.load(train_label_files[i])
        classifier_model.train_on_batch(train_features_chunk, train_labels_chunk)
        print(f"  Processed training chunk {i+1}/{len(train_feature_files)}", end='\r')
    print() # Newline after processing all training chunks

#evaluate model
all_val_features = np.concatenate([np.load(f) for f in val_feature_files])
all_val_labels = np.concatenate([np.load(f) for f in val_label_files])

loss, accuracy = classifier_model.evaluate(all_val_features, all_val_labels)
print(f"Classifier Validation Loss: {loss:.4f}")
print(f"Classifier Validation Accuracy: {accuracy:.4f}")

# save trained model
classifier_model.save("trained_classifier.keras")
classifier_model.save("trained_classifier.h5")
print("Classifier model saved.")

# Clean up the feature chunk files (optional)
# for f in glob.glob(os.path.join(features_dir, "*_features_chunk_*.npy")):
#     os.remove(f)
# for f in glob.glob(os.path.join(features_dir, "*_labels_chunk_*.npy")):
#     os.remove(f)