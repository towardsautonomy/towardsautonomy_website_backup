import os
os.environ['KERAS_BACKEND'] = 'tensorflow'
from cityscape_labels import *
import glob
from os import system

from prep_data import *

import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
import matplotlib.image as mpimg
from keras.models import load_model

test_data_folder = './KITTI/testing/image_2/'
semseg_out_folder = './KITTI/testing_out_ssnet/'

model = load_model('model_ssnet.h5')
img_shape = (1232, 368)

def build_labelmap(prediction, shape, nb_classes):
    labeled_out = np.zeros(shape).astype(np.uint8)
    for i in range(predictions.shape[1]):
        for j in range(predictions.shape[2]):
            for label in range(nb_classes):
                if(predictions[0, i, j, label] > 0.5):
                    labeled_out[i, j, :] = label2rgb[label].color
                
    return labeled_out
        
# Get all the filenames
images = glob.glob(test_data_folder + '*.png')
images.sort()

cmd = 'mkdir -p ' + semseg_out_folder
system(cmd)

for i, fname in enumerate(images):
    # Get the filename without extension
    base=os.path.basename(images[i])
    name=os.path.splitext(base)[0]
    test_semseg_filename = semseg_out_folder + name + ".png"
    test_data = normalized(cv2.resize(cv2.cvtColor(cv2.imread(images[i]), cv2.COLOR_BGR2RGB), img_shape))

    predictions = model.predict(np.expand_dims(test_data, axis=0))
    #gray = predictions[0,:,:,1]
    labeled_out = build_labelmap(predictions, test_data.shape, 34)

    mpimg.imsave(test_semseg_filename, labeled_out)
    print(str(i*100.0/len(images)) + '% completed!') 

print('Predicted SemSeg RGB generated!')