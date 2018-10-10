import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle

class camCalibration:
    def __init__(self, cal_filename=None):
        if cal_filename is None:
            self.cal_filename = "camera_cal/cal_data_pickle.p"
        else:
            self.cal_filename = cal_filename
            
        # Load Calibration data
        fd = open(self.cal_filename,'rb')
        self.cam_cal = pickle.load(fd)
        fd.close()
        
    def calibrateCamera(self, pathname=None, cal_filename=None, nx=9, ny=6):
        if pathname is None:
                pathname = "camera_cal/calibration*.jpg"
        if cal_filename is None:
                cal_filename = self.cal_filename
                
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((ny*nx,3), np.float32)
        objp[:,:2] = np.mgrid[0:nx, 0:ny].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d points in real world space
        imgpoints = [] # 2d points in image plane.

        # Make a list of calibration images
        images = glob.glob(pathname)

        # Step through the list and search for chessboard corners
        for idx, fname in enumerate(images):
            img = cv2.imread(fname)
            img_size = (img.shape[1], img.shape[0])
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (nx,ny), None)

            # If found, add object points, image points
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)

                # Draw and display the corners
                cv2.drawChessboardCorners(img, (nx,ny), corners, ret)
                #write_name = 'corners_found'+str(idx)+'.jpg'
                #cv2.imwrite(write_name, img)
                cv2.imshow('img', img)
                cv2.waitKey(50)

        cv2.destroyAllWindows()

        # Do camera calibration given object points and image points
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size,None,None)


        dst = cv2.undistort(img, mtx, dist, None, mtx)

        # Save the camera calibration result for later use
        dist_pickle = {}
        dist_pickle["mtx"] = mtx
        dist_pickle["dist"] = dist
        pickle.dump(dist_pickle, open(cal_filename, "wb" ))
        
    def undistortImg(self, img):
        image = cv2.undistort(img, self.cam_cal["mtx"], self.cam_cal["dist"], None, self.cam_cal["mtx"])
        return image
            
class laneFinder:
    def __init__(self):
        self.kernel_size = 15
        self.gradx_thres = (30, 100)
        self.grady_thres = (30, 100)
        self.mag_thres = (30, 100)
        self.dir_thres = (3*np.pi/12, 5*np.pi/12) #45deg to 60deg
        self.h_thres = (150, 255)
        self.l_thres= (180, 255)
        self.s_thres = (0, 100)
        
    ## Calculates x or y gradient   
    def abs_sobel_thresh(self, img, orient='x', sobel_kernel=3, thres=(30, 100)):
        # Convert to grayscale
        gray = cv2.cvtColor( img, cv2.COLOR_RGB2GRAY )
        
        # Take the derivative in x or y given orient = 'x' or 'y'
        # Output dtype = cv2.CV_64F. Then take its absolute and convert to cv2.CV_8U
        sobel64f = cv2.Sobel(gray,cv2.CV_64F,orient=='x',orient=='y',ksize=sobel_kernel)
        # Take the absolute value of the derivative or gradient
        abs_sobel64f = np.absolute(sobel64f)
        # Scale to 8-bit (0 - 255) then convert to type = np.uint8
        scaled_sobel = np.uint8(255*abs_sobel64f/np.max(abs_sobel64f))
        
        grad_binary = np.zeros_like(scaled_sobel)
        # Create a mask of 1's where the scaled gradient magnitude is > thresh_min and < thresh_max
        grad_binary[(scaled_sobel > thres[0]) & (scaled_sobel <= thres[1])] = 1
        # Return this mask as the binary_output image
        return grad_binary
        
    ## Calculates gradient magnitude
    def mag_thresh(self, img, sobel_kernel=3, mag_thresh=(30, 100)):
        # Convert to grayscale
        gray = cv2.cvtColor( img, cv2.COLOR_RGB2GRAY )
        # Take the derivative in x direction
        sobelx64f = cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=sobel_kernel)
        # Take the derivative in y direction
        sobely64f = cv2.Sobel(gray,cv2.CV_64F,0,1,ksize=sobel_kernel)
        # Calculate the magnitude 
        abs_sobel64f = np.sqrt(np.power(sobelx64f, 2) + np.power(sobely64f, 2))
        # Scale to 8-bit (0 - 255) then convert to type = np.uint8
        scaled_sobel = np.uint8(255*abs_sobel64f/np.max(abs_sobel64f))
        # Apply threshold
        mag_binary = np.zeros_like(scaled_sobel)
        mag_binary[(scaled_sobel > mag_thresh[0]) & (scaled_sobel <= mag_thresh[1])] = 1
        # Return this mask as the binary_output image
        return mag_binary

    ## Calculates direction gradient
    def dir_threshold(self, img, sobel_kernel=3, thres=(3*np.pi/12, 5*np.pi/12)):
        # Convert to grayscale
        gray = cv2.cvtColor( img, cv2.COLOR_RGB2GRAY )
        # Take the derivative in x direction
        sobelx64f = cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=sobel_kernel)
        # Take the absolute value of the x-derivative or gradient
        abs_sobelx64f = np.absolute(sobelx64f)
        # Take the derivative in y direction
        sobely64f = cv2.Sobel(gray,cv2.CV_64F,0,1,ksize=sobel_kernel)
        # Take the absolute value of the x-derivative or gradient
        abs_sobely64f = np.absolute(sobely64f)
        # Calculate the direction gradient
        dir_grad = np.arctan2(abs_sobely64f, abs_sobelx64f)
        # Apply threshold
        dir_binary = np.zeros_like(dir_grad)
        dir_binary[(dir_grad > thres[0]) & (dir_grad <= thres[1])] = 1
        # Return this mask as the binary_output image
        return dir_binary
        
    ## Calculate color threshold in hls color space
    def hls_threshold(self, rgb, h_thres=(170, 180), l_thres=(180, 255), s_thres=(170, 255)):
        hls = cv2.cvtColor(rgb, cv2.COLOR_RGB2HLS)
        h_channel = hls[:,:,0]
        l_channel = hls[:,:,1]
        s_channel = hls[:,:,2]
        
        h_binary = np.zeros_like(h_channel)
        h_binary[(h_channel > h_thres[0]) & (h_channel <= h_thres[1])] = 1
        
        l_binary = np.zeros_like(l_channel)
        l_binary[(l_channel > l_thres[0]) & (l_channel <= l_thres[1])] = 1
        
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel > s_thres[0]) & (s_channel <= s_thres[1])] = 1
        
        hls_binary = np.zeros_like(h_channel)
        hls_binary[((h_binary == 1) & (s_binary == 1) & (l_binary == 1))] = 1
        
        # Return the hls thresholded binary
        return hls_binary
        
    ## Calculate color threshold in hls color space
    def luv_threshold(self, rgb, l_thres=(170, 180)):
        luv = cv2.cvtColor(rgb, cv2.COLOR_RGB2LUV)
        l_channel = luv[:,:,0]
        u_channel = luv[:,:,1]
        v_channel = luv[:,:,2]
        
        l_binary = np.zeros_like(l_channel)
        l_binary[(l_channel > l_thres[0]) & (l_channel <= l_thres[1])] = 1
        
        # Return the luv thresholded binary
        return l_binary
                
    def select_yellow(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        h_channel = hsv[:,:,0]
        s_channel = hsv[:,:,1]
        v_channel = hsv[:,:,2]
        
        h_thres = (20, 38)
        s_thres = (60, 174)
        v_thres = (60, 250)
        
        h_binary = np.zeros_like(h_channel)
        h_binary[(h_channel > h_thres[0]) & (h_channel <= h_thres[1])] = 1
                
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel > s_thres[0]) & (s_channel <= s_thres[1])] = 1
        
        v_binary = np.zeros_like(v_channel)
        v_binary[(v_channel > v_thres[0]) & (v_channel <= v_thres[1])] = 1
        
        hsv_bin = np.zeros_like(h_channel)
        hsv_bin[((h_binary == 1) & (s_binary == 1) & (v_binary == 1))] = 1

        return hsv_bin

    def select_white(self, image):
        lower = np.array([202,202,202])
        upper = np.array([255,255,255])
        mask = cv2.inRange(image, lower, upper)

        return mask

    ## Combine all the thresholded gradient
    def get_threshold_img(self, image):
        # Apply each of the edge thresholding functions
        gradx = self.abs_sobel_thresh(image, 'x', self.kernel_size, (20, 100))
        grady = self.abs_sobel_thresh(image, 'y', self.kernel_size, (20, 100))
        mag_binary = self.mag_thresh(image, self.kernel_size, (20, 100))
        dir_binary = self.dir_threshold(image, self.kernel_size, (-1*np.pi/6, np.pi/6))
        # Combining above 4 binary images
        combined_edge = np.uint8(np.zeros_like(gradx))
        combined_edge[(gradx == 1) | ((mag_binary == 1) & (dir_binary == 1))] = 1
        # Apply color thresholded image
        
        hls_bin_yellow = self.hls_threshold(image, (15, 40), (0, 255), (70, 255))
        hls_bin_white = self.hls_threshold(image, (0, 255), (225, 255), (0, 255))
        #yellow = self.select_yellow(image)
        #white = self.select_white(image)
        
        # LUV Color Thresholding
        luv_binary = self.luv_threshold(image, (225, 255))
                
        # Combine the two binary thresholds
        combined_binary = np.uint8(np.zeros_like(gradx))
        combined_binary[((hls_bin_white == 1) | (hls_bin_yellow == 1) | (luv_binary == 1))] = 1
        #combined_binary[(combined_edge == 1) & ((hls_bin_yellow == 1) | (hls_bin_white == 1) | (luv_binary == 1))] = 1
        
        # Return the combined thresholded edge    
        return combined_binary 
    
    ## This function uses the internal functions of class 'edgeDetector' and returns the edge image
    def get_lane_img(self, image):
        laneImg = self.get_threshold_img(image)
        return laneImg
        
class imgUnwarper:
    def __init__(self, img):
        self.image = img;
        # Grab the image shape
        self.img_size = (img.shape[1], img.shape[0]) #[x, y]
        
        # Coordinates for source image region
        self.src = np.float32([[np.uint16(self.img_size[0]*0.45), np.uint16(self.img_size[1]*0.625)],
                                [np.uint16(self.img_size[0]*(1-0.45)), np.uint16(self.img_size[1]*0.625)],
                                [0, self.img_size[1]],
                                [self.img_size[0],self.img_size[1]]])
        # Coordinates for destination unwarped image
        self.dst = np.float32([[0, 0], 
                                [self.img_size[0], 0],
                                [0, self.img_size[1]],
                                [self.img_size[0],self.img_size[1]]])
                                        
    ## Unwarp Perspective Transform Matrix    
    def getTransformMatrix(self, src=None, dst=None):
        # Assign default values to src and dst if no values were provided
        if src is None:
            src = self.src
        if dst is None:
            dst = self.dst
        # Given src and dst points, calculate the perspective transform matrix
        M = cv2.getPerspectiveTransform(src, dst)
        return M
        
    ## Inverse Perspective Transform Matrix    
    def getInvTransformMatrix(self, src=None, dst=None):
        # Assign default values to src and dst if no values were provided
        if src is None:
            src = self.src
        if dst is None:
            dst = self.dst
        src = np.float32([[545, 460],
                [735, 460],
                [1280, 700],
                [0, 700]])

        dst = np.float32([[0, 0],
                         [1280, 0],
                         [1280, 720],
                         [0, 720]])
        # Given src and dst points, calculate the perspective transform matrix
        M = cv2.getPerspectiveTransform(dst, src)
        return M
    
    ## Unwarp Image 
    def unwarpImage(self, src=None, dst=None):
        # Assign default values to src and dst if no values were provided
        if src is None:
            src = self.src
        if dst is None:
            dst = self.dst
            
        src = np.float32([[545, 460],
                [735, 460],
                [1280, 700],
                [0, 700]])

        dst = np.float32([[0, 0],
                         [1280, 0],
                         [1280, 720],
                         [0, 720]])
        # Given src and dst points, calculate the perspective transform matrix
        M = self.getTransformMatrix(src, dst)
        # Warp the image using OpenCV warpPerspective()
        warped = cv2.warpPerspective(self.image, M, self.img_size, flags=cv2.INTER_LINEAR)
        # Return the resulting image and matrix
        return warped, M   