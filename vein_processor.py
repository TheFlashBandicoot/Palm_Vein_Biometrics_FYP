

# --------------------------------------- Description --------------------------------------- #

# Python Vein Processor. This python file runs on a Laptop and receives near-infrared
# hand images from a Raspberry Pi IoT device over wifi using a the custom I2P
# appication protocol. It then applies various image processing techniques to extract
# information about the veins in the hand. It then either enrols this information, or 
# attempts to match it to existing information. It returns PASS/FAIL to the IoT device.

# Author: Bryan McSweeney 31/01/21
# Version: 1.0

# ----------------------------------- Imports and variables --------------------------------- #
import socket
import i2p
import sys
import os
import numpy
import pickle
from cv2 import cv2

path = 'C:/Users/bryan/Desktop/College/FYP/Python Server/Hough/x1y1/' # folder to store images/vein data

def convolve2d(image, kernel):
    kernel = numpy.flipud(numpy.fliplr(kernel))
    # convolution output
    output = numpy.zeros_like(image)

    # Add zero padding to the input image
    image_padded = numpy.zeros((image.shape[0] + 2, image.shape[1] + 2))
    image_padded[1:-1, 1:-1] = image

    # Loop over every pixel of the image
    for x in range(image.shape[1]):
        for y in range(image.shape[0]):
            # element-wise multiplication of the kernel and the image
            output[y, x]=(kernel * image_padded[y: y+3, x: x+3]).sum()

    return output
# ------------------------------------- Set up I2P Server ----------------------------------- #
try:
    My_Server = i2p.I2P_Server('',9916)
except socket.error as error:
    print(error)
    My_Server.shutdown()
    sys.exit()


# ----------------------------------------- Main Loop --------------------------------------- #
try:
    while True:

        My_Server.accept()  # Wait for new connection (!! BUG HERE !! CTRL+C does not Trigger KeyboardInterrupt 
                                                        # on windows while scipt is in a blocking wait)
        try:
            command = My_Server.receive()   # command from client (enrol/identify)
            print('Command Received: ' + command)
        except socket.error as error:
            print(error)
            My_Server.close_connection()
            continue    # means client closed connection. go back to accept a new connection

        try:
            image_data_bytes = My_Server.receive()  # Receives image as byte-stream 
        except socket.error as error:
            print(error)
            My_Server.close_connection()
            continue    # means client closed connection. go back to accept a new connection

        image_data_array = numpy.frombuffer(image_data_bytes,dtype=numpy.uint8) # convert to numpy array
        image_data_array = image_data_array.reshape(1232,1640)  # reshape array to match image size

        print('image received successfully') 

        #----------------------------------------------------------------------------
        # Thresholding, contour, Convex hull, hull defects, and mask creation
        #---------------------------------------------------------------------------- 
        ret, silhouette = cv2.threshold(image_data_array,8,255,cv2.THRESH_BINARY) # create silhouette

        kernel = numpy.ones((5,5),numpy.uint8)  # erosion kernel
        mask = cv2.erode(silhouette,kernel,iterations = 1)   # mask creation by erosion

        contours, hierarchy = cv2.findContours(silhouette, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours of silhouette
        contours = max(contours, key=lambda x: cv2.contourArea(x)) # pick out biggest contour

        hull = cv2.convexHull(contours, returnPoints=False,clockwise=True) # construct convex hull
        hull[::-1].sort(axis=0) # sort hull (fix for BUG in the convexHull function)
        defects = cv2.convexityDefects(contours, hull)  # get indices of defects

        #----------------------------------------------------------------------------
        # Finger gap detection and Affline transformations
        #---------------------------------------------------------------------------- 
        cnt = 0
        finger_gaps = []

        for i in range(defects.shape[0]):   # loop over number of rows in defects
            s, e, f, d = defects[i][0] 
            start = tuple(contours[s][0])
            end = tuple(contours[e][0])
            far = tuple(contours[f][0])
            a = numpy.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) # distance formula
            b = numpy.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
            c = numpy.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
            angle = numpy.arccos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))  #      cosine theorem
            if (angle <= numpy.pi / 2) & (a>25) & (b>45):  # angle less than 90 degree, treat as fingers
                finger_gaps.append(far)
                cnt = cnt+1
        
        if (cnt < 4):   # if not enough fingers detected

            print('ERROR: not enough fingers detected..\n\n')
            try:
                My_Server.transmit('FAIL', None)    # Report Failure to IoT Device
            except socket.error as error:
                print(error)
                My_Server.close_connection()
                continue
            except TypeError as error:
                print(error)
                My_Server.close_connection()
            except ValueError as error:
                print(error)
                My_Server.close_connection()

            My_Server.close_connection()
            continue    # go back and wait for new connection

        finger_gap0 = finger_gaps[0]    # seperate each tuple from the list
        finger_gap1 = finger_gaps[1]
        finger_gap2 = finger_gaps[2]
        finger_gap3 = finger_gaps[3]

        if (finger_gap0[0]>finger_gap3[0]): # Determine reference points
            point1 = finger_gap1
            point2 = finger_gap3
        else:
            point1 = finger_gap0
            point2 = finger_gap2

        slope = (point1[1]-point2[1])/(point1[0]-point2[0]) # determine slope of line between reference points
        angle = float(90) - (abs(numpy.arctan(slope))*180/numpy.pi) # determine angle of hand

        if (slope>0):   # determine direction of desired rotation based on slope polarity
            angle = angle*-1

        rot = cv2.getRotationMatrix2D(point1, angle, 1.0)   # generate affline rotation matrix
        image_data_array = cv2.warpAffine(image_data_array, rot, image_data_array.shape[1::-1], flags=cv2.INTER_LINEAR) # rotate image
        mask = cv2.warpAffine(mask, rot, mask.shape[1::-1], flags=cv2.INTER_LINEAR)    # rotate mask

        #----------------------------------------------------------------------------
        # Median filtering and CLAHE
        #----------------------------------------------------------------------------
        image_data_array = cv2.medianBlur(image_data_array, 5)  # apply median blur

        clahe = cv2.createCLAHE(clipLimit=2,tileGridSize=(8,8)) # create CLAHE filter
        image_data_array = clahe.apply(image_data_array) # apply CLAHE

        #----------------------------------------------------------------------------
        #Resizing
        #----------------------------------------------------------------------------
        image_resized = cv2.resize(image_data_array, (410,308), interpolation=cv2.INTER_AREA)
        mask_resized = cv2.resize(mask, (410,308), interpolation=cv2.INTER_AREA)  # resize mask to fit final image

        #----------------------------------------------------------------------------
        # Gaussian Blur
        #----------------------------------------------------------------------------
        image_resized = cv2.GaussianBlur(image_resized, (3,3),0)

        #----------------------------------------------------------------------------
        # Ridge Detection, thresholding, thinning, hand-edge removal, and vein cropping
        #----------------------------------------------------------------------------
        ridge_filter = cv2.ximgproc.RidgeDetectionFilter_create()   # create ridge filter
        image_resized = ridge_filter.getRidgeFilteredImage(image_resized)   # apply ridge filter

        ret,image_resized = cv2.threshold(image_resized,85,255,cv2.THRESH_BINARY)   # apply binary threshold

        image_resized = cv2.ximgproc.thinning(image_resized, thinningType= cv2.ximgproc.THINNING_ZHANGSUEN)

        image_resized = cv2.bitwise_and(image_resized, image_resized, mask=mask_resized)    # apply mask to remove hand-edges

        # crop area where veins should be, based on reference points
        cropped_veins = image_resized[((point1[1]//4)-50):((point2[1]//4)+50), ((point1[0]//4)-20):((point2[0]//4)+160)].copy()

        #----------------------------------------------------------------------------
        # Line Detection with the Probabilistic Hough Transform
        #----------------------------------------------------------------------------
        # lines = cv2.HoughLinesP(cropped_veins,rho=1,theta=numpy.pi/180,threshold=10,minLineLength=10, maxLineGap=6)   # Hough Transform
        # lines = cv2.HoughLinesP(cropped_veins,rho=1,theta=numpy.pi/180,threshold=9,minLineLength=5, maxLineGap=6)   # Hough Transform    <--- THIS ONE

        cropped_veins_BGR = cv2.merge([cropped_veins,cropped_veins,cropped_veins])

        
        # lines_polar = []
        # for line in lines:
        #     x1,y1,x2,y2 = line[0]
        #     cv2.line(cropped_veins_BGR,(x1,y1),(x2,y2),(0,0,255),1)
            
        #     if (y2==y1):
        #         theta = 90
        #         rho = y2
        #     elif (x2==x1):
        #         theta = 0
        #         rho = x2
        #     else:
        #         m = (y2-y1)/(x2-x1)
        #         c = y2-(m*x2)

        #         theta = 90 + (numpy.arctan(m)*180/numpy.pi)
        #         rho = c*numpy.sin(theta)

        #     lines_polar.append([x1,y1,x2,y2,rho,theta])

        cropped_veins_norm = cv2.threshold(cropped_veins,1,1,cv2.THRESH_BINARY)
        bifurcation_kernel = numpy.array([[1, 2, 4],[128, 256, 8],[64, 32, 16]])
        print(cropped_veins_norm)
        print(type(cropped_veins_norm))

        cropped_veins_norm = cropped_veins_norm[1].astype(numpy.uint32)
        print(cropped_veins_norm)

        bifurcation_table = numpy.array([424,394,298,418,297,402,325,340,337,277,330,
                                        420,329,404,293,338,426,341,331,466,436,301,346,
                                        406,421,361,362,425,422,410,458,299,428,434,363,474,438,429])
        # bifurcated_veins = cv2.filter2D(cropped_veins_norm,cv2.CV_16U,bifurcation_kernel)
        # print(bifurcated_veins)

        bifurcated_veins = convolve2d(cropped_veins_norm,bifurcation_kernel)
        print(bifurcated_veins)
        bifurcation_coords = []
        for x in range(bifurcated_veins.shape[1]):
            for y in range(bifurcated_veins.shape[0]):
                for z in bifurcation_table:
                    if (bifurcated_veins[y][x] == z):
                        cv2.circle(cropped_veins_BGR,(x,y),2,[0,0,255],1)
                        bifurcation_coords.append([x,y])

        #----------------------------------------------------------------------------
        # Enrollment/Identification
        #----------------------------------------------------------------------------

        if (command == '   enrol'):
            
            try:
                My_Server.transmit('PASS', None)    # Report PASS to IoT Device
            except socket.error as error:
                print(error)
                My_Server.close_connection()
                continue
            except TypeError as error:
                print(error)
                My_Server.close_connection()
            except ValueError as error:
                print(error)
                My_Server.close_connection()

            My_Server.close_connection()

            # name = input('Please enter name of individual: ')
            # with open(path+name+'.p','wb') as f:
            #     pickle.dump(lines_polar, f) # save line data in binary file

            # cv2.imwrite(path+name+'_img.png',cropped_veins_BGR)
            # cv2.imshow('cropped_veins_BGR', cropped_veins_BGR)
            # cv2.waitKey(0)
            # continue    # go back and wait for new connection
        
        elif (command == 'identify'):

            # for filename in os.listdir(path):
            #     if filename.endswith(".p"):
            #         with open(path+filename,'rb') as f:
            #             enrolled_lines = pickle.load(f) # load line data from binary file

            #         match = 0
            #         # similarity = 0
            #         # lines_polar_count = 0
            #         # enrolled_lines_count = 0
            #         # multiplier1 = 1
            #         # multiplier2 = 1

            #         # for [x1,y1,x2,y2,rho,theta] in lines_polar:
            #         # #     # if (lines_polar_count == 30):
            #         # #         # break
            #         #     for [x11,y11,x21,y21,rho1, theta1] in enrolled_lines:
            #         # #         # if (enrolled_lines_count == 50):
            #         # #         #     break
            #         # #         multiplier1 = 1
            #         # #         multiplier2 = 1
            #         #         if ((abs(rho-rho1)<3) and (abs(theta-theta1)<3)):
            #         # #             if ( (lines_polar_count<=(len(lines_polar)/2)) and (lines_polar_count>(len(lines_polar)/4)) ):
            #         # #                 multiplier1 = 0.75
            #         # #             elif ( (lines_polar_count<=(3*(len(lines_polar)/4))) and (lines_polar_count>(len(lines_polar)/2)) ):
            #         # #                 multiplier1 = 0.5
            #         # #             elif ( (lines_polar_count<=len(lines_polar)) and (lines_polar_count>(3*(len(lines_polar)/4))) ):
            #         # #                 multiplier1 = 0.25
            #         #             if ((abs(x1-x11)<10) and (abs(x2-x21)<10)):

            #         #                 match = match + 1
            #         #                 break
            #         # #             if ( (enrolled_lines_count<=(len(enrolled_lines)/2)) and (enrolled_lines_count>(len(enrolled_lines)/4)) ):
            #         # #                 multiplier2 = 0.75
            #         # #             elif ( (enrolled_lines_count<=(3*(len(enrolled_lines)/4))) and (enrolled_lines_count>(len(enrolled_lines)/2)) ):
            #         # #                 multiplier2 = 0.5
            #         # #             elif ( (enrolled_lines_count<=len(enrolled_lines)) and (enrolled_lines_count>(3*(len(enrolled_lines)/4))) ):
            #         # #                 multiplier2 = 0.25

            #         # #             match = match + (multiplier1*multiplier2)
            #         # #             break
            #         # #         enrolled_lines_count = enrolled_lines_count + 1

            #         # #     lines_polar_count = lines_polar_count + 1

            #         for [rho,theta] in lines_polar:
            #             for [rho1,theta1] in enrolled_lines:
            #                 if ((abs(rho-rho1)<3) and (abs(theta-theta1)<3)):
            #                     match = match + 1
            #                     break

            #         similarity = match/len(lines_polar)
                    
            #         print(filename)
            #         # # print(enrolled_lines)
            #         # # print('\n\n\n')
            #         # # print(lines_polar)
            #         print(match)
            #         print(similarity)

            
            try:
                My_Server.transmit('PASS', None)    # Report PASS to IoT Device
            except socket.error as error:
                print(error)
                My_Server.close_connection()
                continue
            except TypeError as error:
                print(error)
                My_Server.close_connection()
            except ValueError as error:
                print(error)
                My_Server.close_connection()

            My_Server.close_connection()

            cv2.imshow('cropped_veins_BGR', cropped_veins_BGR)
            cv2.waitKey(0)
            continue    # go back and wait for new connection



except KeyboardInterrupt:
    print('CTRL+C input detected. Closing Server Application...')
    My_Server.shutdown()
    sys.exit()