from keras.models import Sequential
from keras.layers.core import Activation, Flatten, Dense, Dropout, Reshape
from keras.layers.convolutional import Conv2D, MaxPooling2D, ZeroPadding2D, UpSampling2D
from keras import regularizers
from keras.layers.normalization import BatchNormalization
import cv2, numpy as np

def SS_Net(input_shape=(368, 1232, 3), classes=34):
    model = Sequential()
    
    # Encoder
    model.add(Conv2D(64, kernel_size=(5, 5), strides=(1, 1), padding="same", input_shape=input_shape))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(64, kernel_size=(5, 5), strides=(1, 1), padding="same", input_shape=input_shape))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    model.add(Conv2D(128, kernel_size=(5, 5), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(128, kernel_size=(5, 5), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    model.add(Conv2D(256, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(256, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(256, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D((2,2), strides=(2,2)))

    model.add(Conv2D(512, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(512, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(512, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D((2,2), strides=(2,2)))
    model.add(Dropout(0.2))

    # Decoder    
    model.add(UpSampling2D((2,2)))
    model.add(Conv2D(512, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(512, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(512, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Dropout(0.2))
    
    model.add(UpSampling2D((2,2)))
    model.add(Conv2D(256, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(256, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(256, kernel_size=(3, 3), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    model.add(UpSampling2D((2,2)))
    model.add(Conv2D(128, kernel_size=(5, 5), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(128, kernel_size=(5, 5), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    model.add(UpSampling2D((2,2))) 
    model.add(Conv2D(64, kernel_size=(5, 5), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Conv2D(64, kernel_size=(5, 5), strides=(1, 1), padding="same"))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    model.add(Conv2D(filters=classes, kernel_size=(3, 3), strides=(1, 1), padding="same", activation='softmax'))
    model.add(Reshape((input_shape[0], input_shape[1], classes)))
    return model