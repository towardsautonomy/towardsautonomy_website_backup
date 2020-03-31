import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
from sklearn.model_selection import train_test_split
from predictor import *

predictor = vehPredictor()

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
#sample_size = min(len(car_images), len(notcar_images))
sample_size = 5
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
from keras.models import load_model

predictions = []
for i in range(X.shape[0]):
    predict = predictor.predict(np.expand_dims(X[i], axis=0))
    predictions.append(np.uint8(predict[0]))
print(predictions)
print(y)
test_err = predictions - y

plt.figure
plt.plot(test_err)
plt.title('Test Error')

plt.show()