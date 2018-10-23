import os
os.environ['KERAS_BACKEND'] = 'tensorflow'

import glob
import numpy as np
import cv2
from prep_data import *
from segnet import SegNet, preprocess_input, to_categorical
import timeit
from ssnet import *
import tensorflow as tf

from keras import backend as K
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config = config)
    
K.tensorflow_backend._get_available_gpus()

train_data, train_label = get_data_from_img()
#train_label = np.reshape(train_label,(367,data_shape,12))

input_shape_1 = (360, 480, 3)
nb_classes_1 = 12

input_shape_2 = (368, 1232, 3)
nb_classes_2 = 34

nb_epoch = 1000
batch_size = 2
#X = preprocess_input(X_train)
#Y = to_categorical(Y, nb_classes)

model = SS_Net(input_shape=input_shape_2, classes=nb_classes_2)

from keras.optimizers import SGD
sgd = SGD(lr=0.01, decay=1e-6, momentum=0.9, nesterov=True)
#model.compile(loss='categorical_crossentropy', optimizer=sgd, metrics=["accuracy"])
model.compile(loss="categorical_crossentropy", optimizer='adadelta', metrics=["accuracy"])

tic=timeit.default_timer()
model.fit(train_data, train_label, batch_size=batch_size, epochs=nb_epoch)
toc=timeit.default_timer()
elapsed_time = toc - tic
model.save('model.h5')
print('Total time taken for training: ', elapsed_time/60.0, 'secs')

'''
### plot the training and validation loss for each epoch
plt.figure
plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')

plt.show()
'''