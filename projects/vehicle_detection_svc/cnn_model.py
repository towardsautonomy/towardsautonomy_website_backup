import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
from sklearn.model_selection import train_test_split

# Read in cars and notcars
cars = []
notcars = []


car_images = glob.glob('dataset/vehicles/KITTI_extracted/*.png');
for image in car_images:
    cars.append(image)

notcar_images = glob.glob('dataset/non-vehicles/Extras/*.png');
for image in notcar_images:
    notcars.append(image)
        
# Reduce the sample size because
# The quiz evaluator times out after 13s of CPU time
sample_size = min(len(car_images), len(notcar_images))
#sample_size = 500
cars = cars[0:sample_size]
notcars = notcars[0:sample_size]

images = []
for car in cars:
    car_images = cv2.imread(car)
    car_images = cv2.cvtColor(car_images, cv2.COLOR_BGR2YUV)
    images.append(car_images)
    
for notcar in notcars:
    notcar_images = cv2.imread(notcar)
    notcar_images = cv2.cvtColor(notcar_images, cv2.COLOR_BGR2YUV)
    images.append(notcar_images)

# Create an array stack of feature vectors
X = np.array(images)

# Define the labels vector
y = np.array(np.hstack((np.ones(len(cars)), np.zeros(len(cars)))))

from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda
from keras.layers import Conv2D, MaxPooling2D, Cropping2D, Dropout
from keras.optimizers import Adam

# Sequential model
model = Sequential()
# Crop the top 50 rows and bottom 20 rows to have the learning focus on roads; Output = 3@(90, 320)
#model.add(Cropping2D(cropping=((50, 20),(0, 0)), input_shape=(160,320,3)))
## Creating a Deep Neural Network Architecture. Reference: "End to End Learning for Self-Driving Cars" by NVIDIA
#   https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf
# Normalization Layer: Normalize and mean shift to 0
model.add(Lambda(lambda x: x/255.0 - 0.5, input_shape=(64,64,3)))
# Layer 1: Input - 3@(90, 320); Output - 24@(43, 158)
model.add(Conv2D(filters=24,kernel_size=(5,5),strides=(2,2),padding='valid',activation="relu"))
# Layer 2: Input - 24@(43, 158); Output - 36@(20, 77)
model.add(Conv2D(filters=36,kernel_size=(5,5),strides=(2,2),padding='valid',activation="relu"))
# Layer 3: Input - 36@(20, 77); Output - 48@(8, 37)
model.add(Conv2D(filters=48,kernel_size=(5,5),strides=(2,2),padding='valid',activation="relu"))
# Layer 4: Input - 48@(8, 37); Output - 64@(6, 35)
model.add(Conv2D(filters=64,kernel_size=(3,3),strides=(1,1),padding='valid',activation="relu"))
# Layer 5: Input - 64@(6, 35); Output - 64@(4, 33)
model.add(Conv2D(filters=64,kernel_size=(3,3),strides=(1,1),padding='valid',activation="relu"))
# Fully Connected Layers
# Layer 6: Input - 64@(4, 33); Output - 8448
model.add(Flatten())
# Layer 7: Output - 100
model.add(Dense(100, activation="relu"))
model.add(Dropout(0.7))
# Layer 8: Output - 50
model.add(Dense(50, activation="relu"))
model.add(Dropout(0.7))
# Output - 1
model.add(Dense(1, activation='sigmoid'))

optimizer = Adam(lr=0.007)
model.compile(loss='binary_crossentropy', optimizer=optimizer)
history_object = model.fit(X, y, batch_size=256, validation_split=0.2, shuffle=True, epochs=20, verbose=1)

model.save('model.h5')

### print the keys contained in the history object
print(history_object.history.keys())

### plot the training and validation loss for each epoch
plt.figure(1)
plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')

plt.show()