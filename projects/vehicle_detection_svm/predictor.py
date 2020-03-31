import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
from sklearn.model_selection import train_test_split

from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda
from keras.layers import Conv2D, MaxPooling2D, Cropping2D, Dropout
from keras.optimizers import Adam
from keras.models import load_model

class vehPredictor:
    def __init__(self):
        self.model = load_model('model.h5')
        
    def predict(self, X):
        predictions = self.model.predict(X)
        # round predictions
        rounded_pred = [round(x[0]) for x in predictions]
        return rounded_pred