import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from moviepy.editor import VideoFileClip
from imgProcessor import *
from vehDetectFunc import *
import pickle
from scipy.ndimage.measurements import label

VIS_WINDOWS = False
VIS_BBOX = False
VIS_HEAT = False
TEST_ON_IMAGE = True

# Define a function to visualize search window
def get_windows(img, window_size, ystart, ystop, scale, pix_per_cell, cell_per_block):
    draw_img = np.copy(img)
    
    img_tosearch = img[ystart:ystop,:,:]
    ctrans_tosearch = convert_color(img_tosearch, conv='RGB2YCrCb')
    if scale != 1:
        imshape = ctrans_tosearch.shape
        ctrans_tosearch = cv2.resize(ctrans_tosearch, (np.int(imshape[1]/scale), np.int(imshape[0]/scale)))
        
    ch1 = ctrans_tosearch[:,:,0]
    ch2 = ctrans_tosearch[:,:,1]
    ch3 = ctrans_tosearch[:,:,2]

    # Define blocks and steps as above
    nxblocks = (ch1.shape[1] // pix_per_cell) - cell_per_block + 1
    nyblocks = (ch1.shape[0] // pix_per_cell) - cell_per_block + 1 

    
    # 64 was the orginal sampling rate, with 8 cells and 8 pix per cell
    nblocks_per_window = (window_size // pix_per_cell) - cell_per_block + 1
    cells_per_step = 2  # Instead of overlap, define how many cells to step
    nxsteps = (nxblocks - nblocks_per_window) // cells_per_step + 1
    nysteps = (nyblocks - nblocks_per_window) // cells_per_step + 1
    
    # Bounding Boxes
    bbox_list = []
    
    for xb in range(nxsteps):
        for yb in range(nysteps):
            ypos = yb*cells_per_step
            xpos = xb*cells_per_step

            xleft = xpos*pix_per_cell
            ytop = ypos*pix_per_cell
            
            xbox_left = np.int(xleft*scale)
            ytop_draw = np.int(ytop*scale)
            win_draw = np.int(window_size*scale)
            
            top_left = (xbox_left, ytop_draw+ystart)
            bottom_right = (xbox_left+win_draw,ytop_draw+win_draw+ystart)
            # Append bbox position to list
            bbox_list.append((top_left, bottom_right))
                
    # Return the list of bounding boxes
    return bbox_list
    
# Define a single function that can extract features using hog sub-sampling and make predictions
def find_cars(img, window_size, ystart, ystop, scale, svc, X_scaler, orient, pix_per_cell, cell_per_block, spatial_size, hist_bins):
    
    draw_img = np.copy(img)
    
    img_tosearch = img[ystart:ystop,:,:]
    ctrans_tosearch = convert_color(img_tosearch, conv='RGB2YCrCb')
    if scale != 1:
        imshape = ctrans_tosearch.shape
        ctrans_tosearch = cv2.resize(ctrans_tosearch, (np.int(imshape[1]/scale), np.int(imshape[0]/scale)))
        
    ch1 = ctrans_tosearch[:,:,0]
    ch2 = ctrans_tosearch[:,:,1]
    ch3 = ctrans_tosearch[:,:,2]

    # Define blocks and steps as above
    nxblocks = (ch1.shape[1] // pix_per_cell) - cell_per_block + 1
    nyblocks = (ch1.shape[0] // pix_per_cell) - cell_per_block + 1 
    nfeat_per_block = orient*cell_per_block**2
    
    # 64 was the orginal sampling rate, with 8 cells and 8 pix per cell
    nblocks_per_window = (window_size // pix_per_cell) - cell_per_block + 1
    cells_per_step = 1  # Instead of overlap, define how many cells to step
    nxsteps = (nxblocks - nblocks_per_window) // cells_per_step + 1
    nysteps = (nyblocks - nblocks_per_window) // cells_per_step + 1
    
    # Compute individual channel HOG features for the entire image
    hog1 = get_hog_features(ch1, orient, pix_per_cell, cell_per_block, feature_vec=False)
    hog2 = get_hog_features(ch2, orient, pix_per_cell, cell_per_block, feature_vec=False)
    hog3 = get_hog_features(ch3, orient, pix_per_cell, cell_per_block, feature_vec=False)
    
    # Bounding Boxes
    bbox_list = []
    
    for xb in range(nxsteps):
        for yb in range(nysteps):
            ypos = yb*cells_per_step
            xpos = xb*cells_per_step
            # Extract HOG for this patch
            hog_feat1 = hog1[ypos:ypos+nblocks_per_window, xpos:xpos+nblocks_per_window].ravel() 
            hog_feat2 = hog2[ypos:ypos+nblocks_per_window, xpos:xpos+nblocks_per_window].ravel() 
            hog_feat3 = hog3[ypos:ypos+nblocks_per_window, xpos:xpos+nblocks_per_window].ravel() 
            hog_features = np.hstack((hog_feat1, hog_feat2, hog_feat3))

            xleft = xpos*pix_per_cell
            ytop = ypos*pix_per_cell

            # Extract the image patch
            subimg = cv2.resize(ctrans_tosearch[ytop:ytop+window_size, xleft:xleft+window_size], (64,64))
          
            # Get color features
            spatial_features = bin_spatial(subimg, size=spatial_size)
            hist_features = color_hist(subimg, nbins=hist_bins)

            # Scale features and make a prediction
            test_features = X_scaler.transform(np.hstack((spatial_features, hist_features, hog_feat1)).reshape(1, -1))    
            #test_features = X_scaler.transform(np.hstack((shape_feat, hist_feat)).reshape(1, -1))    
            test_prediction = svc.predict(test_features)
            
            if test_prediction == 1:
                xbox_left = np.int(xleft*scale)
                ytop_draw = np.int(ytop*scale)
                win_draw = np.int(window_size*scale)
                
                top_left = (xbox_left, ytop_draw+ystart)
                bottom_right = (xbox_left+win_draw,ytop_draw+win_draw+ystart)
                # Append bbox position to list
                bbox_list.append((top_left, bottom_right))
                
    # Return the list of bounding boxes
    return bbox_list

def add_heat(heatmap, bbox_list):
    # Iterate through list of bboxes
    for box in bbox_list:
        # Add += 1 for all pixels inside each bbox
        # Assuming each "box" takes the form ((x1, y1), (x2, y2))
        heatmap[box[0][1]:box[1][1], box[0][0]:box[1][0]] += 1

    # Return updated heatmap
    return heatmap# Iterate through list of bboxes
    
def apply_threshold(heatmap, threshold):
    # Zero out pixels below the threshold
    heatmap[heatmap <= threshold] = 0
    # Return thresholded map
    return heatmap

def draw_labeled_bboxes(img, labels):
    # Iterate through all detected cars
    for car_number in range(1, labels[1]+1):
        # Find pixels with each car_number label value
        nonzero = (labels[0] == car_number).nonzero()
        # Identify x and y values of those pixels
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Define a bounding box based on min/max x and y
        bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
        # Draw the box on the image
        cv2.rectangle(img, bbox[0], bbox[1], (255,0,0), 6)
    # Return the image
    return img

def vehicleDetection_pipeline(image):
    # Undistort the image
    img = cc.undistortImg(image)
    heat = np.zeros_like(img[:,:,0]).astype(np.float)

    window_size = 64
    bbox_list = []

    if(VIS_WINDOWS == True):
        ystart = 370
        ystop = 470
        scale = 0.5
        bboxes = get_windows(img, window_size, ystart, ystop, scale, pix_per_cell, cell_per_block)
        bbox_list.extend(bboxes)
        
        ystart = 370
        ystop = 500
        scale = 1
        bboxes = get_windows(img, window_size, ystart, ystop, scale, pix_per_cell, cell_per_block)
        bbox_list.extend(bboxes)
      
        ystart = 370
        ystop = 600
        scale = 1.5
        bboxes = get_windows(img, window_size, ystart, ystop, scale, pix_per_cell, cell_per_block)
        bbox_list.extend(bboxes)
      
        ystart = img.shape[0]/2
        ystop = img.shape[0]
        scale = 2
        bboxes = get_windows(img, window_size, ystart, ystop, scale, pix_per_cell, cell_per_block)
        bbox_list.extend(bboxes)
        
        ystart = img.shape[0]/2
        ystop = img.shape[0]
        scale = 2.5
        bboxes = get_windows(img, window_size, ystart, ystop, scale, pix_per_cell, cell_per_block)
        bbox_list.extend(bboxes)
        
        out_img = draw_boxes(img, bbox_list)
        return out_img
    else:  
        ystart = 370
        ystop = 470
        scale = 0.5
        bboxes = find_cars(img, window_size, ystart, ystop, scale, svc, X_scaler, orient, pix_per_cell, cell_per_block, spatial_size, hist_bins)
        bbox_list.extend(bboxes)
        
        ystart = 370
        ystop = 500
        scale = 1
        bboxes = find_cars(img, window_size, ystart, ystop, scale, svc, X_scaler, orient, pix_per_cell, cell_per_block, spatial_size, hist_bins)
        bbox_list.extend(bboxes)
       
        ystart = 370
        ystop = 600
        scale = 1.5
        bboxes = find_cars(img, window_size, ystart, ystop, scale, svc, X_scaler, orient, pix_per_cell, cell_per_block, spatial_size, hist_bins)
        bbox_list.extend(bboxes)
        
        ystart = img.shape[0]/2
        ystop = img.shape[0]
        scale = 2
        bboxes = find_cars(img, window_size, ystart, ystop, scale, svc, X_scaler, orient, pix_per_cell, cell_per_block, spatial_size, hist_bins)
        bbox_list.extend(bboxes)
        
        ystart = img.shape[0]/2
        ystop = img.shape[0]
        scale = 2.5
        bboxes = find_cars(img, window_size, ystart, ystop, scale, svc, X_scaler, orient, pix_per_cell, cell_per_block, spatial_size, hist_bins)
        bbox_list.extend(bboxes)
    
    if(VIS_BBOX == True):
        draw_img = draw_boxes(img, bbox_list)
        
    else:
        # Add heat to each box in box list
        heat = add_heat(heat,bbox_list)
            
        # Apply threshold to help remove false positives
        heat = apply_threshold(heat,7)

        # Visualize the heatmap when displaying    
        heatmap = np.clip(heat, 0, 255)

        # Find final boxes from heatmap using label function
        labels = label(heatmap)
        
        draw_img = draw_labeled_bboxes(np.copy(img), labels)
        
        if(VIS_HEAT == True):
            fig = plt.figure()
            plt.imshow(heatmap, cmap='hot')
            plt.title('Heat Map')
            plt.show()
    
    return draw_img
    
if __name__=="__main__":
    # load a pree-trained svc model from a serialized (pickle) file
    dist_pickle = pickle.load( open("trainsvc_pickle.p", "rb" ) )

    # get attributes of our svc object
    svc = dist_pickle["svc"]
    X_scaler = dist_pickle["scaler"]
    orient = dist_pickle["orient"]
    pix_per_cell = dist_pickle["pix_per_cell"]
    cell_per_block = dist_pickle["cell_per_block"]
    spatial_size = dist_pickle["spatial_size"]
    hist_bins = dist_pickle["hist_bins"]

    # Read in an image
    image = mpimg.imread('test_images/test5.jpg')    
    
    # Create an instance of Camera Calibration Class
    cc = camCalibration("camera_cal/cal_data_pickle.p")
            
    if((VIS_HEAT == True) or (TEST_ON_IMAGE == True)):
        ## Test image and save
        detectedVeh = vehicleDetection_pipeline(image)
        if(TEST_ON_IMAGE == True):
            mpimg.imsave('output_images/test5.jpg', detectedVeh)
            
        fig = plt.figure()
        plt.imshow(detectedVeh)
        plt.title('Detected Vehicle')
        plt.show()
        
    else:
        out_clip_filename = "project_video_out.mp4"
        clip1 = VideoFileClip("project_video.mp4")
        processed_clip = clip1.fl_image(vehicleDetection_pipeline) #NOTE: this function expects color images!!
        
        if((VIS_WINDOWS == True) or (VIS_BBOX == True)):
            processed_clip.preview()
        else:
            processed_clip.write_videofile(out_clip_filename, audio=False)
            #processed_clip.preview()