import glob
import numpy as np
import cv2
from os import system
import os

def normalized(rgb):
    #return rgb/255.0
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    norm=np.zeros(hsv.shape, np.float32)

    norm[:,:,0]=(hsv[:,:,0]/180.0) - 0.5
    norm[:,:,1]=(hsv[:,:,1]/255.0) - 0.5
    norm[:,:,2]=(hsv[:,:,2]/255.0) - 0.5

    return norm
    
def normalized_augmented(rgb):
    #return rgb/255.0
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    norm=np.zeros(hsv.shape, np.float32)

    v=hsv[:,:,2]

    norm[:,:,0]=(hsv[:,:,0]/180.0) - 0.5
    norm[:,:,1]=(hsv[:,:,1]/255.0) - 0.5
    norm[:,:,2]=(cv2.equalizeHist(v)/255.0) - 0.5

    return norm

def bin_prep(labels, dim_xy = (480,360), n_classes = 12):
    x = np.zeros([dim_xy[1],dim_xy[0],n_classes])
    for i in range(dim_xy[1]):
        for j in range(dim_xy[0]):
            x[i,j,labels[i,j]]=1
    return x

def get_data_from_txt(path = './CamVid/'):
    train_data = []
    train_label = []
    import os
    with open(path+'train.txt') as f:
        txt = f.readlines()
        txt = [line.split(' ') for line in txt]
    for i in range(len(txt)):
        train_data.append(normalized(cv2.imread(os.getcwd() + txt[i][0][7:])))
        train_label.append(bin_prep(cv2.imread(os.getcwd() + txt[i][1][7:][:-1])[:,:,0], (480,360), 12))
        
        train_data.append(normalized_augmented(cv2.imread(os.getcwd() + txt[i][0][7:])))
        train_label.append(bin_prep(cv2.imread(os.getcwd() + txt[i][1][7:][:-1])[:,:,0], (480,360), 12))
        
        print('.',end='')
    return np.array(train_data), np.array(train_label)
    
def get_data_from_img(train_data_folder = './KITTI/training/image_2/', label_data_folder = './KITTI/training/semantic/'):
    train_data = []
    train_label = []
    
    img_shape = (1232, 368)
    
    # Get all the filenames
    images = glob.glob(train_data_folder + '*.png')
    images.sort()
    
    for i, fname in enumerate(images):
        # Get the filename without extension
        base=os.path.basename(images[i])
        name=os.path.splitext(base)[0]
        label_filename = label_data_folder + name + ".png"
        train_data.append(normalized(cv2.resize(cv2.cvtColor(cv2.imread(images[i]), cv2.COLOR_BGR2RGB), img_shape)))
        train_label.append(bin_prep(cv2.resize(cv2.imread(label_filename, cv2.IMREAD_GRAYSCALE), img_shape), img_shape, 34))
        
        train_data.append(normalized_augmented(cv2.resize(cv2.cvtColor(cv2.imread(images[i]), cv2.COLOR_BGR2RGB), img_shape)))
        train_label.append(bin_prep(cv2.resize(cv2.imread(label_filename, cv2.IMREAD_GRAYSCALE), img_shape), img_shape, 34))
        print('.',end='')
    return np.array(train_data), np.array(train_label)