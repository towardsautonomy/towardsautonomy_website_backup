import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from moviepy.editor import VideoFileClip
from imgProcessor import *
  
LANEWIDTH = 3.7
DOTTED_LANE_LEN = 3.0
N_PIXELS_BETWEEN_LANES = 840
N_PIXELS_DOTTED_LANES_LEN = 120
leftLaneEdgeCut = False
rightLaneEdgeCut = False
ROC_past = np.zeros((1, 25), dtype='float')

class lane:
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False  
        #average x values of the fitted line over the last n iterations
        self.bestx = None     
        #polynomial coefficients averaged over the last n iterations
        self.best_fit = [np.array([False])]    
        #polynomial coefficients for the most recent fit
        self.current_fit = [np.array([False])]  
        #polynomial coefficients for past fits
        self.past_fit = np.zeros((5, 3), dtype='float') 
        #radius of curvature of the line in some units
        self.radius_of_curvature = None
        #distance in meters of vehicle center from the line
        self.line_base_pos = None 
        #First Loop
        self.firstLoop = True
        
    def reset(self):
        self.detected = False
        self.firstLoop = True
        self.past_fit = np.zeros((5, 3), dtype='float')
        self.roc_past = []

def textOnImage(image, position, text, color=(255,255,255)):
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = position
    fontScale              = 0.8
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

def window_mask(width, height, img_ref, center,level):
    output = np.zeros_like(img_ref)
    output[int(img_ref.shape[0]-(level+1)*height):int(img_ref.shape[0]-level*height),max(0,int(center-width/2)):min(int(center+width/2),img_ref.shape[1])] = 1
    return output

def find_window_centroids(image, window_width, window_height, stride):
    
    window_centroids = [] # Store the (left,right) window centroid positions per level
    left_window_centroids = []
    right_window_centroids = []
    leftLaneEdgeCut = False
    rightLaneEdgeCut = False
    
    window = np.ones(window_width) # Create our window template that we will use for convolutions
    
    # First find the two starting positions for the left and right lane by using np.sum to get the vertical image slice
    # and then np.convolve the vertical image slice with the window template 
    
    # Sum quarter bottom of image to get slice, could use a different ratio
    # l_sum is a vector of shape 1x(image.shape[1]/2) where each value is the sum of the vertical slice
    l_sum = np.sum(image[int(3*image.shape[0]/4):image.shape[0],0:int(image.shape[1]/2)], axis=0)
    l_center = np.argmax(np.convolve(window,l_sum))-window_width/2
    l_center_prev = l_center
    r_sum = np.sum(image[int(3*image.shape[0]/4):image.shape[0],int(image.shape[1]/2):image.shape[1]], axis=0)
    r_center = np.argmax(np.convolve(window,r_sum))-window_width/2+int(image.shape[1]/2)
    r_center_prev = r_center
    
    # Add what we found for the first layer
    window_centroids.append((l_center,r_center))
    left_window_centroids.append(l_center)
    right_window_centroids.append(r_center)
    
    # Go through each layer looking for max pixel locations
    for level in range(1,(int)(image.shape[0]/window_height)):
        # convolve the window into the vertical slice of the image
        image_layer = np.sum(image[int(image.shape[0]-(level+1)*window_height):int(image.shape[0]-level*window_height),:], axis=0)
        conv_signal = np.convolve(window, image_layer)
        # Find the best left centroid by using past left center as a reference
        # Use window_width/2 as offset because convolution signal reference is at right side of window, not center of window
        offset = window_width/2
        l_min_index = int(max(l_center+offset-stride,0))
        l_max_index = int(min(l_center+offset+stride,image.shape[1]))
        if((l_max_index <= (offset - stride)) or (l_min_index == (image.shape[1] >= (offset - stride)))):
            leftLaneEdgeCut = True
            
        if(max(conv_signal[l_min_index:l_max_index]) > 10):
            l_center = np.argmax(conv_signal[l_min_index:l_max_index])+l_min_index-offset
            l_center_prev = l_center
        else:
            if(((l_center + (l_center - l_center_prev)) > offset) and ((l_center + (l_center - l_center_prev)) < (image.shape[1] - offset))):
                l_center = l_center + (l_center - l_center_prev)
            else:
                leftLane.reset()
            
        # Find the best right centroid by using past right center as a reference
        r_min_index = int(max(r_center+offset-stride,0))
        r_max_index = int(min(r_center+offset+stride,image.shape[1]))
        if((r_min_index <= (offset - stride)) or (r_max_index >= (image.shape[1] - (offset - stride)))):
            rightLaneEdgeCut = True
            
        if(max(conv_signal[r_min_index:r_max_index]) > 10):
            r_center = np.argmax(conv_signal[r_min_index:r_max_index])+r_min_index-offset
            r_center_prev = r_center
        else:
            if(((r_center + (r_center - r_center_prev)) > offset) and ((r_center + (r_center - r_center_prev)) < (image.shape[1] - offset))):
                r_center = r_center + (r_center - r_center_prev)
            else:
                rightLane.reset()
            
        # Add what we found for that layer
        if leftLaneEdgeCut == False:
            left_window_centroids.append(l_center)
        if rightLaneEdgeCut == False:
            right_window_centroids.append(r_center)
        window_centroids.append((l_center,r_center))
    return (left_window_centroids, right_window_centroids)

def laneDetection_pipeline(image):
    # Undistort the image
    img = cc.undistortImg(image)
    # Create an instance of Image Unwarper
    iu = imgUnwarper(img)
    # Get the birds eye view of road by unwarping image
    birdsEyeImg, perspective_M = iu.unwarpImage()
    # Create an instance of Edge Detector
    lf = laneFinder()
    laneImg = lf.get_lane_img(birdsEyeImg)
    out_img = np.dstack((laneImg, laneImg, laneImg))*255

    # window settings
    window_width = 50 
    window_height = 50 # Break image into 9 vertical layers since image height is 720
    stride = 100 # How much to slide left and right for searching
    
    leftx = []
    lefty = []
    rightx = []
    righty = []
    
    if((leftLane.detected == False) or (rightLane.detected == False)):
        (left_window_centroids, right_window_centroids) = find_window_centroids(laneImg, window_width, window_height, stride)
     
        # If we found any window centers
        if ((len(left_window_centroids) > 0) and (len(right_window_centroids) > 0)):
            leftLane.detected = True
            rightLane.detected = True
            # Points used to draw all the left and right windows
            l_points = np.zeros_like(laneImg)
            r_points = np.zeros_like(laneImg)


            # Go through each level and draw the windows 	
            for level in range(0,len(left_window_centroids)):
                # Window_mask is a function to draw window areas
                l_mask = window_mask(window_width,window_height,laneImg,left_window_centroids[level],level)
                # Add graphic points from window mask here to total pixels found 
                l_points[(l_points == 255) | ((l_mask == 1) ) ] = 255
                leftx.append(left_window_centroids[level])
                lefty.append(img.shape[0]-(level+1)*window_height + window_height/2)
                
            for level in range(0,len(right_window_centroids)):
                # Window_mask is a function to draw window areas
                r_mask = window_mask(window_width,window_height,laneImg,right_window_centroids[level],level)
                # Add graphic points from window mask here to total pixels found
                r_points[(r_points == 255) | ((r_mask == 1) ) ] = 255
                rightx.append(right_window_centroids[level])
                righty.append(img.shape[0]-(level+1)*window_height + window_height/2)
                
                
            # Fit a second order polynomial to each
            leftLane.current_fit = np.polyfit(lefty, leftx, 2)
            rightLane.current_fit = np.polyfit(righty, rightx, 2)
            
            template = np.array(r_points+l_points,np.uint8) # add both left and right window pixels together
            zero_channel = np.zeros_like(template) # create a zero color channel
            template = np.array(cv2.merge((zero_channel,template,zero_channel)),np.uint8) # make window pixels green
            warpage= np.dstack((laneImg, laneImg, laneImg))*255 # making the original road pixels 3 color channels
            output = cv2.addWeighted(warpage, 1, template, 0.2, 0.0) # overlay the orignal road image with window results
            
        else:
            # Display orginal road image for the first frame
            output = np.dstack((laneImg, laneImg, laneImg))*255
        return output
        resizedRow = 200          
        resizedCol = 200
        resized_image = cv2.resize(output, (resizedRow, resizedCol)) 
        # Draw Boundary
        resized_image[:, 0] = 255
        resized_image[:, resizedCol-1] = 255
        resized_image[0, :] = 255
        resized_image[resizedRow - 1, :] = 255
        
        overlay_img = np.zeros_like(img)
        offset = 50
        for i in range(offset, resizedRow+offset):
            for j in range(offset, resizedCol+offset):
                overlay_img[i, j, :] = resized_image[i-offset, j-offset, :]
                           
        
        # Create an image to draw the lines on
        color_warp = np.zeros_like(img)

        M = iu.getInvTransformMatrix()
        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, M, (image.shape[1], image.shape[0])) 
        # Combine the result with the original image
        result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)
        output = cv2.addWeighted(result, 1, overlay_img, 1, 0)
        
        textOnImage(output, (300,100), 'Lane Confidence: Low', (255,0,0))
        
       
    else:
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = laneImg.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        left_lane_inds = ((nonzerox > (leftLane.current_fit[0]*(nonzeroy**2) + leftLane.current_fit[1]*nonzeroy + 
                            leftLane.current_fit[2] - margin)) & (nonzerox < (leftLane.current_fit[0]*(nonzeroy**2) + 
                            leftLane.current_fit[1]*nonzeroy + leftLane.current_fit[2] + margin))) 

        right_lane_inds = ((nonzerox > (rightLane.current_fit[0]*(nonzeroy**2) + rightLane.current_fit[1]*nonzeroy + 
                            rightLane.current_fit[2] - margin)) & (nonzerox < (rightLane.current_fit[0]*(nonzeroy**2) + 
                            rightLane.current_fit[1]*nonzeroy + rightLane.current_fit[2] + margin)))  

        # Again, extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        
        # Fit a second order polynomial to each
        if(len(leftx) > 10):
            leftLane.current_fit = np.polyfit(lefty, leftx, 2)
        else:
            leftLane.reset()
        
        if(len(rightx) > 10):        
            rightLane.current_fit = np.polyfit(righty, rightx, 2)
        else:
            rightLane.reset()
                
        leftLane.past_fit[0,:] = leftLane.current_fit
        rightLane.past_fit[0,:] = rightLane.current_fit
        
        # Get the best fit by averaging the left lane poly fit
        if(leftLane.firstLoop == True):
            leftLane.firstLoop = False
            for i in range(1, 5):
                leftLane.past_fit[i,:] = leftLane.current_fit
        else:
            for i in range(4, 0, -1):
                leftLane.past_fit[i,:] = leftLane.past_fit[i - 1,:]
                
        if(leftLane.firstLoop == False):
            leftLane.best_fit = 0.3*leftLane.past_fit[0, :] + \
                                0.3*leftLane.past_fit[1, :] + \
                                0.2*leftLane.past_fit[2, :] + \
                                0.1*leftLane.past_fit[3, :] + \
                                0.1*leftLane.past_fit[4, :]
            
        # Get the best fit by averaging the right lane poly fit
        if(rightLane.firstLoop == True):
            rightLane.firstLoop = False
            for i in range(1, 5):
                rightLane.past_fit[i,:] = rightLane.current_fit
        else:
            for i in range(4, 0, -1):
                rightLane.past_fit[i,:] = rightLane.past_fit[i - 1,:]
            
        if(rightLane.firstLoop == False):
            rightLane.best_fit = 0.3*rightLane.past_fit[0, :] + \
                                 0.3*rightLane.past_fit[1, :] + \
                                 0.2*rightLane.past_fit[2, :] + \
                                 0.1*rightLane.past_fit[3, :] + \
                                 0.1*rightLane.past_fit[4, :]
        
        # Generate x and y values for plotting
        ploty = (np.linspace(0, img.shape[0]-1, img.shape[0] ))
        
        
        leftLane.bestx = np.int16(np.zeros(ploty.shape[0]))
        leftLane.besty = np.int16(ploty)
        rightLane.bestx = np.int16(np.zeros(ploty.shape[0]))
        rightLane.besty = np.int16(ploty)
        
        leftLane.bestx = leftLane.best_fit[0]*ploty**2 + leftLane.best_fit[1]*ploty + leftLane.best_fit[2]
        rightLane.bestx = rightLane.best_fit[0]*ploty**2 + rightLane.best_fit[1]*ploty + rightLane.best_fit[2]
        
        # Make the left lane pixels Red, right lane pixels Blue
        colored_lanes = np.zeros_like(img)
        colored_lanes[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        colored_lanes[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
        
        # Draw a line at the center of the detected lanes
        polyline = np.zeros_like(img)
        yellow = [255, 255, 0]
        bottomLaneSeparation = rightLane.bestx[-1] - leftLane.bestx[-1]
        extrapolateLeft = False
        extrapolateRight = False
        if((len(leftx) > 50) and (len(rightx) < 50)):
            extrapolateRight = True
        elif((len(leftx) < 50) and (len(rightx) > 50)):
            extrapolateLeft= True
        elif((len(leftx) < 50) and (len(rightx) < 50)):
            leftLane.reset()
            rightLane.reset()
        
        # Execute if the extrapolate flag was not set
        if((extrapolateLeft == False) and (extrapolateRight == False)):
            for i in range(0, ploty.shape[0]):
                if((leftLane.bestx[i] > 0) and (leftLane.bestx[i] < img.shape[1]-1)):
                    leftLane.besty[i] = (np.int16(ploty[i]))
                    polyline[np.int16(ploty[i]), np.int16(leftLane.bestx[i])] = yellow
                    for k in range(1, 5):
                        polyline[np.int16(ploty[i]), np.int16(leftLane.bestx[i]-k)] = yellow
                
                if((rightLane.bestx[i] > 0) and (rightLane.bestx[i] < img.shape[1]-1)):
                    rightLane.besty[i] = (np.int16(ploty[i]))
                    polyline[np.int16(ploty[i]), np.int16(rightLane.bestx[i])] = yellow
                    for k in range(1, 5):
                        polyline[np.int16(ploty[i]), np.int16(rightLane.bestx[i]-k)] = yellow
            
                if(((leftLane.bestx[i] > 0) and (leftLane.bestx[i] < img.shape[1]-1)) and
                    ((rightLane.bestx[i] > 0) and (rightLane.bestx[i] < img.shape[1]-1))):
                    if(((rightLane.bestx[i] - leftLane.bestx[i]) > (N_PIXELS_BETWEEN_LANES + (N_PIXELS_BETWEEN_LANES/2 - 100))) or
                        ((rightLane.bestx[i] - leftLane.bestx[i]) < (N_PIXELS_BETWEEN_LANES - (N_PIXELS_BETWEEN_LANES/2 - 100)))):
                        if((len(leftx)>50) or (len(rightx)>50)): 
                            if(len(leftx) > len(rightx)):
                                extrapolateRight = True
                            else:
                                extrapolateLeft = True   
                            break
                        else:
                            leftLane.reset()
                            rightLane.reset()
                            break
                    
        # Perform extrapolation if extrapolate flag was set
        if((extrapolateLeft == True) or(extrapolateRight == True)):
            for i in range(0, ploty.shape[0]):
                if((extrapolateLeft == True) and (bottomLaneSeparation < (N_PIXELS_BETWEEN_LANES + 250)) and (bottomLaneSeparation > (N_PIXELS_BETWEEN_LANES - 250))):
                    leftLane.bestx[i] = (rightLane.bestx[i] - bottomLaneSeparation)
                elif((extrapolateRight == True) and (bottomLaneSeparation < (N_PIXELS_BETWEEN_LANES + 250)) and (bottomLaneSeparation > (N_PIXELS_BETWEEN_LANES - 250))):
                    rightLane.bestx[i] = (leftLane.bestx[i] + bottomLaneSeparation)
                else:
                    if((extrapolateLeft == True) or (extrapolateRight == True)):
                        extrapolateLeft = False
                        extrapolateRight = False
                        leftLane.reset()
                        rightLane.reset()
                        break          
                        
                if(((leftLane.bestx[i] > 0) and (leftLane.bestx[i] < img.shape[1]-1)) and
                    ((rightLane.bestx[i] > 0) and (rightLane.bestx[i] < img.shape[1]-1))):
                    if(((rightLane.bestx[i] - leftLane.bestx[i]) > (N_PIXELS_BETWEEN_LANES + (N_PIXELS_BETWEEN_LANES/2 - 100))) or
                        ((rightLane.bestx[i] - leftLane.bestx[i]) < (N_PIXELS_BETWEEN_LANES - (N_PIXELS_BETWEEN_LANES/2 - 100)))):
                        leftLane.reset()
                        rightLane.reset()
                        break
                
        # Define conversions in x and y from pixels space to meters
        ym_per_pix = DOTTED_LANE_LEN/N_PIXELS_DOTTED_LANES_LEN # meters per pixel in y dimension
        xm_per_pix = LANEWIDTH/N_PIXELS_BETWEEN_LANES # meters per pixel in x dimension
        y_eval = np.max(ploty)
        scaledY = ploty*ym_per_pix
        scaledLeftX = leftLane.bestx*xm_per_pix
        scaledRightX = rightLane.bestx*xm_per_pix
        
        leftLane.line_base_pos = xm_per_pix*(leftLane.bestx[-1] - img.shape[1]/2)
        rightLane.line_base_pos = xm_per_pix*(rightLane.bestx[-1] - img.shape[1]/2)
        laneCenter = (leftLane.line_base_pos + rightLane.line_base_pos)/2
        #print('Lane Center = ' + str(laneCenter))

        # Fit new polynomials to x,y in world space
        left_fit_cr = np.polyfit(leftLane.besty*ym_per_pix, scaledLeftX, 2)
        right_fit_cr = np.polyfit(ploty*ym_per_pix, scaledRightX, 2)
        # Calculate the new radii of curvature
        leftLane.radius_of_curvature = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        rightLane.radius_of_curvature = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
        # Now our radius of curvature is in meters
        STRAIGHT_LINE_ROC = 10000
        if(leftLane.radius_of_curvature > STRAIGHT_LINE_ROC): leftLane.radius_of_curvature = STRAIGHT_LINE_ROC
        if(rightLane.radius_of_curvature > STRAIGHT_LINE_ROC): rightLane.radius_of_curvature = STRAIGHT_LINE_ROC
        if((leftLane.bestx[0] < leftLane.bestx[-1]) and (rightLane.bestx[0] < rightLane.bestx[-1])):
            radiusOfCurvature = -1*(leftLane.radius_of_curvature + rightLane.radius_of_curvature)/2
        elif(((leftLane.bestx[0] < leftLane.bestx[-1]) and (rightLane.bestx[0] > rightLane.bestx[-1])) or
             ((leftLane.bestx[0] > leftLane.bestx[-1]) and (rightLane.bestx[0] < rightLane.bestx[-1]))):
             radiusOfCurvature = STRAIGHT_LINE_ROC
        else:
            radiusOfCurvature = (leftLane.radius_of_curvature + rightLane.radius_of_curvature)/2
            
        ROC_past[0, 0] = radiusOfCurvature
        for i in range(ROC_past.shape[1]-1, 0, -1):
            if(ROC_past[0, i-1] == 0):
                ROC_past[0, i] = radiusOfCurvature
            else:
                ROC_past[0, i] = ROC_past[0, i-1]
                
        ROC_avg = 0
        for i in range(0, ROC_past.shape[1]):
            ROC_avg = ROC_avg + ROC_past[0, i]
        ROC_avg = ROC_avg/ROC_past.shape[1]
        
        #print(leftLane.radius_of_curvature, 'm', rightLane.radius_of_curvature, 'm')
                        
        # Generate a polygon to illustrate the search window area
        # And recast the x and y points into usable format for cv2.fillPoly()
        left_line_window1 = np.array([np.transpose(np.vstack([leftLane.bestx-margin, ploty]))])
        left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([leftLane.bestx+margin, 
                                      ploty])))])
        left_line_pts = np.hstack((left_line_window1, left_line_window2))
        right_line_window1 = np.array([np.transpose(np.vstack([rightLane.bestx-margin, ploty]))])
        right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([rightLane.bestx+margin, 
                                      ploty])))])
        right_line_pts = np.hstack((right_line_window1, right_line_window2))

        # Draw the lane onto the warped blank img
        window_img = np.zeros_like(img)
        cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
        cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
        # Draw the results
        birdsEyeOut = cv2.addWeighted(window_img, 0.2, colored_lanes, 1, 0.0) # overlay the orignal road image with window results
        birdsEyeOut = cv2.addWeighted(birdsEyeOut, 1, polyline, 1, 0.0) # overlay the polyfit line
        
        resizedRow = 200
        resizedCol = 200
        resized_image = cv2.resize(birdsEyeOut, (resizedRow, resizedCol)) 
        # Draw Boundary
        resized_image[:, 0] = 255
        resized_image[:, resizedCol-1] = 255
        resized_image[0, :] = 255
        resized_image[resizedRow - 1, :] = 255
        
        overlay_img = np.zeros_like(img)
        offset = 50
        for i in range(offset, resizedRow+offset):
            for j in range(offset, resizedCol+offset):
                overlay_img[i, j, :] = resized_image[i-offset, j-offset, :]
                                    
        # Create an image to draw the lines on
        color_warp = np.zeros_like(img)

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([leftLane.bestx, leftLane.besty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([rightLane.bestx, rightLane.besty])))])
        pts = np.hstack((pts_left, pts_right))
      
        if((leftLane.detected == True) and (rightLane.detected == True)):
            # Draw the lane onto the warped blank image
            if((extrapolateLeft == True) or(extrapolateRight == True)):
                cv2.fillPoly(color_warp, np.int_([pts]), (255,255,0))
            else:
                cv2.fillPoly(color_warp, np.int_([pts]), (0,255,255))

        M = iu.getInvTransformMatrix()
        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, M, (img.shape[1], img.shape[0])) 
        # Combine the result with the original image
        result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)
        output = cv2.addWeighted(result, 1, overlay_img, 1, 0)
        
        if((leftLane.detected == True) and (rightLane.detected == True)):
            if(extrapolateLeft == True):
                textOnImage(output, (300,100), 'Lane Confidence: Medium', (255,255,0))
                textOnImage(output, (300,190), 'Left Lane Extrapolated', (255,255,0))
            elif(extrapolateRight == True):
                textOnImage(output, (300,100), 'Lane Confidence: Medium', (255,255,0))
                textOnImage(output, (300,190), 'Right Lane Extrapolated', (255,255,0))
            else:
                textOnImage(output, (300,100), 'Lane Confidence: High', (0,0,255))
            if(ROC_avg >= 0):
                ROC_Text = 'Radius of Curvature: ' + str(ROC_avg) + ' meters [Clockwise]'
            else:
                ROC_Text = 'Radius of Curvature: ' + str(abs(ROC_avg)) + ' meters [Anti-Clockwise]'
            textOnImage(output, (300,130), ROC_Text, (0,0,255))
            laneCenterText = 'Lane Center Offset = ' + str(laneCenter) + ' meters'
            textOnImage(output, (300,160), laneCenterText, (0,0,255))
            
        else:
            textOnImage(output, (300,100), 'Lane Confidence: Low', (255,0,0))

    return output
      
leftLane = lane()
rightLane = lane()      
if __name__=="__main__":
    # Read in an image
    image = mpimg.imread('test_images/straight_lines1.jpg')    
    
    # Create an instance of Camera Calibration Class
    cc = camCalibration("camera_cal/cal_data_pickle.p")
    
    out_clip_filename = "test_lane_car.mp4"
    clip1 = VideoFileClip("test_video_out.mp4")
    processed_clip = clip1.fl_image(laneDetection_pipeline) #NOTE: this function expects color images!!
    processed_clip.write_videofile(out_clip_filename, audio=False)
    #processed_clip.preview()
    
    '''
    ## Test image and save
    edge = laneDetection_pipeline(image)
    #edge = laneDetection_pipeline(image)
    #mpimg.imsave('output_images/straight_lines1.jpg', edge)
    
    # Display the edge image
    plt.figure
    plt.imshow(edge, cmap='gray')
    plt.title('Sliding Window Detection Output')
    plt.show()
    '''
    