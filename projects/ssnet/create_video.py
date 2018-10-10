import numpy as np
import cv2
from PIL import Image
import matplotlib.image as mpimg
from os import system
import glob
import argparse
import matplotlib.pyplot as plt
from moviepy.editor import VideoFileClip
from moviepy.editor import ImageSequenceClip
import math
import os

data_folder = 'KITTI/testing/image_2/'
label_folder = 'KITTI/testing_out_ssnet/'
vid_frame_dir = 'KITTI/vid_frame/'
vid_filename = 'output.mp4'
img_shape = (1232, 368)

generate_frames = 0

def textOnImage(image, position, text, color=(255,255,255)):
    font                   = cv2.FONT_HERSHEY_COMPLEX
    bottomLeftCornerOfText = position
    fontScale              = 1.5
    fontColor              = color
    thickness              = 2
    lineType               = 2

    cv2.putText(image,text, 
        bottomLeftCornerOfText, 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)

def write_video(images, fps, out_filename):
    print("Creating video {}, FPS={}".format(out_filename, fps))
    clip = ImageSequenceClip(images, fps=fps)
    
    clip.write_videofile(out_filename)
    
if __name__ == '__main__': 
    if(generate_frames == 1):
        # Get all the filenames
        labels = glob.glob(label_folder + '*.png')
        labels.sort()

        for i, fname in enumerate(labels):
            # Get the filename without extension
            base=os.path.basename(labels[i])
            name=os.path.splitext(base)[0]
            label_img = cv2.cvtColor(cv2.imread(labels[i]), cv2.COLOR_BGR2RGB)
            raw_img = cv2.resize(cv2.cvtColor(cv2.imread(data_folder + name + '.png'), cv2.COLOR_BGR2RGB), img_shape)
            road = np.all(label_img == (128, 64,128), axis=-1)
            person = np.all(label_img == (220, 20, 60), axis=-1)
            rider = np.all(label_img == (255,  0,  0), axis=-1)
            cars = np.all(label_img == (  0,  0,142), axis=-1)
            trucks = np.all(label_img == (  0,  0, 70), axis=-1)
            bus = np.all(label_img == (  0, 60,100), axis=-1)
            motorcycle = np.all(label_img == (  0,  0,230), axis=-1)
            license_plate = np.all(label_img == (  0,  0,142), axis=-1)
            bicycle = np.all(label_img == (119, 11, 32), axis=-1)
            for j in range(raw_img.shape[0]):
                for k in range(raw_img.shape[1]):
                    if(road[j,k] == 1):
                        raw_img[j,k,1] = 255    # Make Roads green
                    elif((person[j,k] == 1) or (rider[j,k] == 1) or (cars[j,k] == 1) or (trucks[j,k] == 1) or (bus[j,k] == 1) or (motorcycle[j,k] == 1) or (license_plate[j,k] == 1) or (bicycle[j,k] == 1)):
                        raw_img[j,k,0] = 255    # Make Objects red
            
            cv2.imwrite(vid_frame_dir + name + ".png", cv2.cvtColor(raw_img, cv2.COLOR_RGB2BGR))
            print('Wrote --> ' + vid_frame_dir + name + ".png")
    
    # Create video from frames
    images = glob.glob(vid_frame_dir + '*.png')
    images.sort()
    FPS = 7
    write_video(images, FPS, vid_filename)