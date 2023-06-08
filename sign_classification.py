import os
import matplotlib.pyplot as plt
import numpy as np

import cv2

import tensorflow as tf

from tensorflow import keras
from tensorflow.keras import regularizers


print("TensorFlow version:", tf.__version__)

class_names = ['background', 'hardstop', 'speedlimit', 'speedlimitoff', 'stop']
batch_size = 64
image_size = (64, 64)
input_shape = image_size + (1, )
print(input_shape)
seed = 321
validation_split = 0.1

data_augmentation = tf.keras.Sequential([
  tf.keras.layers.RandomRotation(0.2, fill_mode="nearest", seed=seed),
  tf.keras.layers.RandomZoom(0.1, 0.1, fill_mode="nearest", seed=seed),
  tf.keras.layers.RandomBrightness((-0.3, 0.15), seed=seed),
  tf.keras.layers.RandomContrast(0.1, seed=seed),
])

ds_training = tf.keras.preprocessing.image_dataset_from_directory(
    './training',
    labels='inferred',
    label_mode='int',
    class_names=class_names,
    batch_size=batch_size,
    image_size=(240, 320),
    color_mode="grayscale",
    shuffle=True,
    seed=seed,
    validation_split=validation_split,
    subset='training',
)

ds_training = ds_training.map(lambda x, y: (data_augmentation(x, training=True), y), num_parallel_calls=tf.data.AUTOTUNE)

ds_validation = tf.keras.preprocessing.image_dataset_from_directory(
    './training',
    labels='inferred',
    label_mode='int',
    class_names=class_names,
    batch_size=batch_size,
    image_size=(240, 320),
    color_mode="grayscale",
    shuffle=True,
    seed=seed,
    validation_split=validation_split,
    subset='validation',
)

#ds_validation = ds_validation.map(lambda x, y: (data_augmentation(x, training=False), y), num_parallel_calls=tf.data.AUTOTUNE)

model = tf.keras.models.Sequential([
  tf.keras.layers.Resizing(image_size[0], image_size[1]),
  tf.keras.layers.Cropping2D(cropping=((int(image_size[0] * 0.5), 0), (int(image_size[1] * 0.1), int(image_size[1] * 0.1)))),
  tf.keras.layers.Conv2D(32, (3, 3), padding="same", activation="relu"),
  tf.keras.layers.MaxPooling2D(2, 2),
  tf.keras.layers.Conv2D(64, (3, 3), padding="same", activation="relu"),
  tf.keras.layers.MaxPooling2D(2, 2),
  tf.keras.layers.Conv2D(128, (3, 3), padding="same", activation="relu"),
  tf.keras.layers.MaxPooling2D(2, 2),
  tf.keras.layers.Flatten(input_shape=input_shape),
  tf.keras.layers.Dense(128, activation='relu', kernel_regularizer=regularizers.l2(0.0002)),
  tf.keras.layers.Dense(128, activation='relu', kernel_regularizer=regularizers.l2(0.0002)),
  tf.keras.layers.Dropout(0.25),
  tf.keras.layers.Dense(5)
])

loss_fn = tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True)

model.compile(optimizer="adam",
              loss=loss_fn,
              metrics=['accuracy'])

if True:
  model.fit(ds_training, epochs=50, verbose=2)
  model.save('saved_model2')
else:
  model = tf.keras.models.load_model('saved_model2')

model.summary()

model.evaluate(ds_validation, verbose=2)

if False:
  probability_model = tf.keras.Sequential([
    model,
    tf.keras.layers.Softmax()
  ])

  for images, labels in ds_validation.take(10):
      test_images = images[:64]
      test_labels = labels[:64]

  predicted_probabilities = probability_model.predict(test_images)

  predicted_labels = tf.argmax(predicted_probabilities, axis=-1)

  for i in range(len(test_labels)):
      print("Predicted label:", class_names[predicted_labels[i].numpy()])
      print("True label:", class_names[test_labels[i].numpy()])
      print("Confidence:", max(predicted_probabilities[i]))
      print("------------------")
