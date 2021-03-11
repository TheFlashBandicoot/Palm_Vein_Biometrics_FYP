
# Server to receive images from pi over i2p and process them

import socket
import i2p
import sys
import numpy
import pickle
from PIL import Image
from cv2 import cv2

image_path = 'C:/Users/bryan/Desktop/College/FYP/Python Server/Format_Testing/'

try:
    My_Server = i2p.I2P_Server('',9916)
except socket.error as error:
    print(error)
    My_Server.shutdown()
    sys.exit()

try:
    while True:

        My_Server.accept()

        try:
            command = My_Server.receive()
            print('Command Received: ' + command)
        except socket.error as error:
            print(error)
            My_Server.close_connection()
            continue

        try:
            image_data_bytes = My_Server.receive()
        except socket.error as error:
            print(error)
            My_Server.close_connection()
            continue    # means client closed connection. go back to accept a new connection

        # image_filename = input('Please enter filename for image: ')
        image_filename = 'format_test_binned_temp2'
        full_path = image_path + image_filename + '.png'

        # image_file = open(full_path, 'wb')
        # image_file.write(image_data_bytes)
        # image_file.close()

        image_data_array = numpy.frombuffer(image_data_bytes,dtype=numpy.uint8)
        image_data_array = image_data_array.reshape(1232,1640)
        # pillow_image_data = Image.fromarray(image_data_array, 'L')
        # pillow_image_data.show()
        # pillow_image_data.save(full_path)

        print('image received successfully')
        

        try:
            if (command=='identify'):
                My_Server.transmit('PASS', None)
            else:
                My_Server.transmit('FAIL', None)
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

        cv2.imshow('image_data_array', image_data_array)
        cv2.waitKey(100)

        #----------------------------------------------------------------------------
        # Thresholding, Convex hull, hull defects and mask creation
        #---------------------------------------------------------------------------- 
        ret, silhouette = cv2.threshold(image_data_array,8,255,cv2.THRESH_BINARY) # create silhouette
        
        cv2.imshow('silhouette', silhouette)
        cv2.waitKey(100)

        kernel = numpy.ones((5,5),numpy.uint8)  # erosion kernel
        erosion = cv2.erode(silhouette,kernel,iterations = 1)   # mask creation by erosion
        cv2.imshow('erosion', erosion)
        cv2.waitKey(100)

        image_data_array_copy = numpy.copy(image_data_array)  # create copy of original image for drawing on
        image_data_array_BGR = cv2.merge([image_data_array_copy,image_data_array_copy,image_data_array_copy])

        contours, hierarchy = cv2.findContours(silhouette, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours of silhouette
        contours = max(contours, key=lambda x: cv2.contourArea(x)) # pick out biggest contour
        cv2.drawContours(image_data_array_BGR, [contours], -1, (255,30,0), 2)   # draw in blue
        cv2.imshow("contours", image_data_array_BGR)
        cv2.waitKey(100)

        hull = cv2.convexHull(contours, returnPoints=False,clockwise=True) # construct convex hull
        print(hull)
        hull[::-1].sort(axis=0) # sort hull (fix for bug in the function)
        print(hull)
        hullxy = cv2.convexHull(contours)    # get hull again but the x,y coordinates
        defects = cv2.convexityDefects(contours, hull)  # get indices of defects
        cv2.drawContours(image_data_array_BGR, [hullxy], -1, (30,0,255), 2)  # draw hull
        cv2.imshow("hull", image_data_array_BGR)
        cv2.waitKey(100)

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
        
        finger_gap0 = finger_gaps[0]
        finger_gap1 = finger_gaps[1]
        finger_gap2 = finger_gaps[2]
        finger_gap3 = finger_gaps[3]

        if (finger_gap0[0]>finger_gap3[0]):
            point1 = finger_gap1
            point2 = finger_gap3
            cv2.circle(image_data_array_BGR, finger_gap0, 20, [0,0,255], -1)
            cv2.circle(image_data_array_BGR, finger_gap1, 20, [0,255,0], -1)
            cv2.circle(image_data_array_BGR, finger_gap2, 20, [0,0,255], -1)
            cv2.circle(image_data_array_BGR, finger_gap3, 20, [0,255,0], -1)
            cv2.line(image_data_array_BGR,finger_gap1,finger_gap3,[0,255,0],2)
        else:
            point1 = finger_gap0
            point2 = finger_gap2
            cv2.circle(image_data_array_BGR, finger_gap0, 20, [0,255,0], -1)
            cv2.circle(image_data_array_BGR, finger_gap1, 20, [0,0,255], -1)
            cv2.circle(image_data_array_BGR, finger_gap2, 20, [0,255,0], -1)
            cv2.circle(image_data_array_BGR, finger_gap3, 20, [0,0,255], -1)
            cv2.line(image_data_array_BGR,finger_gap0,finger_gap2,[0,255,0],2)

        cv2.imshow("finger detect", image_data_array_BGR)
        cv2.waitKey(100)

        # midpoint = (((point1[0]+point2[0])/2),((point1[1]+point2[1])/2))
        slope = (point1[1]-point2[1])/(point1[0]-point2[0])
        print(slope)
        angle = float(90) - (abs(numpy.arctan(slope))*180/numpy.pi)
        
        if (slope>0):
            angle = angle*-1
        
        rot = cv2.getRotationMatrix2D(point1, angle, 1.0)
        image_data_array_rotated = cv2.warpAffine(image_data_array, rot, image_data_array.shape[1::-1], flags=cv2.INTER_LINEAR)
        erosion_rotated = cv2.warpAffine(erosion, rot, erosion.shape[1::-1], flags=cv2.INTER_LINEAR)
        cv2.imshow("rotated", image_data_array_rotated)
        cv2.waitKey(100)


        #----------------------------------------------------------------------------
        # Median filtering and CLAHE
        #----------------------------------------------------------------------------
        image_median = cv2.medianBlur(image_data_array_rotated, 5)

        clahe = cv2.createCLAHE(clipLimit=2,tileGridSize=(8,8))
        image_clahe = clahe.apply(image_median)
        # cv2.imshow('image_clahe', image_clahe)
        # cv2.waitKey(100)


        #----------------------------------------------------------------------------
        #Resizing
        #----------------------------------------------------------------------------
        # image_resized1 = cv2.resize(image_clahe, (820,616), interpolation=cv2.INTER_AREA)
        image_resized2 = cv2.resize(image_clahe, (410,308), interpolation=cv2.INTER_AREA)
        erosion_resized = cv2.resize(erosion_rotated, (410,308), interpolation=cv2.INTER_AREA)  # resize mask to fit final image
        # cv2.imshow('image_resized1', image_resized1)
        # cv2.waitKey(100)
        cv2.imshow('image_resized2', image_resized2)
        cv2.waitKey(100)


        #----------------------------------------------------------------------------
        # Gaussian Blur
        #----------------------------------------------------------------------------
        # image_blurred = cv2.GaussianBlur(image_clahe, (7,7),0)
        # image_blurred_R1 = cv2.GaussianBlur(image_resized1, (5,5),0)
        image_blurred_R2 = cv2.GaussianBlur(image_resized2, (3,3),0)

        # #----------------------------------------------------------------------------
        # # Adaptive Binary Thresholding
        # #----------------------------------------------------------------------------
        # image_thresh = cv2.adaptiveThreshold(image_blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,39, 2)
        # image_thresh_R1 = cv2.adaptiveThreshold(image_blurred_R1, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,19, 2)
        # image_adapt_thresh_R2 = cv2.adaptiveThreshold(image_blurred_R2, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,9, 2)

        # cv2.imshow('image_thresh',image_thresh)
        # cv2.waitKey(100)
        # cv2.imshow('image_thresh_R1',image_thresh_R1)
        # cv2.waitKey(100)
        # cv2.imshow('image_adapt_thresh_R2',image_adapt_thresh_R2)
        # cv2.waitKey(100)


        # #----------------------------------------------------------------------------
        # # Sobel + thresholding
        # #----------------------------------------------------------------------------
        # image_sobel = cv2.Sobel(image_blurred, cv2.CV_64F,1,1,ksize=3)
        # min = numpy.min(image_sobel)
        # image_sobel = image_sobel - min #to have only positive values
        # max = numpy.max(image_sobel) 
        # div = max / float(255) #calculate the normalize divisor
        # image_sobel_8u = numpy.uint8(numpy.round(image_sobel / div))
        # cv2.imshow('image_sobel_8u', image_sobel_8u)
        # cv2.waitKey(100)

        # image_sobel_R1 = cv2.Sobel(image_blurred_R1, cv2.CV_64F,1,1,ksize=3)
        # min = numpy.min(image_sobel_R1)
        # image_sobel_R1 = image_sobel_R1 - min #to have only positive values
        # max = numpy.max(image_sobel_R1) 
        # div = max / float(255) #calculate the normalize divisor
        # image_sobel_8u_R1 = numpy.uint8(numpy.round(image_sobel_R1 / div))
        # cv2.imshow('image_sobel_8u_R1', image_sobel_8u_R1)
        # cv2.waitKey(100)

        # image_sobel_R2 = cv2.Sobel(image_blurred_R2, cv2.CV_64F,1,1,ksize=3)
        # min = numpy.min(image_sobel_R2)
        # image_sobel_R2 = abs(image_sobel_R2) #to have only positive values
        # max = numpy.max(image_sobel_R2) 
        # div = max / float(255) #calculate the normalize divisor
        # image_sobel_8u_R2 = numpy.uint8(numpy.round(image_sobel_R2 / div))
        # cv2.imshow('image_sobel_8u_R2', image_sobel_8u_R2)
        # cv2.waitKey(100)

        # ret,sobel_thresh = cv2.threshold(image_sobel_8u,120,255,cv2.THRESH_BINARY)
        # ret1,sobel_thresh_R1 = cv2.threshold(image_sobel_8u_R1,120,255,cv2.THRESH_BINARY)
        # ret2,sobel_thresh_R2 = cv2.threshold(image_sobel_8u_R2,20,255,cv2.THRESH_BINARY)
        # # image_thresh = cv2.adaptiveThreshold(image_sobel_8u, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,39, 2)
        # # image_thresh_R1 = cv2.adaptiveThreshold(image_sobel_8u_R1, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,19, 2)
        # image_thresh_R2 = cv2.adaptiveThreshold(image_sobel_8u_R2, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,9, 2)
        # cv2.imshow('sobel_thresh', sobel_thresh)
        # cv2.waitKey(100)
        # cv2.imshow('image_thresh_R2', image_thresh_R2)
        # cv2.waitKey(100)
        # cv2.imshow('sobel_thresh_R2', sobel_thresh_R2)
        # cv2.waitKey(100)


        # #----------------------------------------------------------------------------
        # # Canny Edge Detection
        # #----------------------------------------------------------------------------
        # image_canny = cv2.Canny(image_clahe, 30, 40, apertureSize=3, L2gradient=True)
        # cv2.imshow('image_canny', image_canny)
        # cv2.waitKey(100)
        # image_canny_R1 = cv2.Canny(image_resized1, 30, 40, apertureSize=3, L2gradient=True)
        # cv2.imshow('image_canny_R1', image_canny_R1)
        # cv2.waitKey(100)
        # image_canny_R2 = cv2.Canny(image_resized2, 30, 40, apertureSize=3, L2gradient=True)
        # cv2.imshow('image_canny_R2', image_canny_R2)
        # cv2.waitKey(100)

        #----------------------------------------------------------------------------
        # Ridge Detection
        #----------------------------------------------------------------------------
        ridge_filter = cv2.ximgproc.RidgeDetectionFilter_create()
        # image_ridge = ridge_filter.getRidgeFilteredImage(image_blurred)
        # image_ridge_R1 = ridge_filter.getRidgeFilteredImage(image_blurred_R1)
        image_ridge_R2 = ridge_filter.getRidgeFilteredImage(image_blurred_R2)
        # cv2.imshow('image_ridge', image_ridge)
        # cv2.waitKey(100)
        # cv2.imshow('image_ridge_R1', image_ridge_R1)
        # cv2.waitKey(100)
        cv2.imshow('image_ridge_R2', image_ridge_R2)
        cv2.waitKey(100)

        # ret,ridge_thresh = cv2.threshold(image_ridge,30,255,cv2.THRESH_BINARY)
        # ret,ridge_thresh_R1 = cv2.threshold(image_ridge_R1,60,255,cv2.THRESH_BINARY)
        ret,ridge_thresh_R2 = cv2.threshold(image_ridge_R2,85,255,cv2.THRESH_BINARY)
        # cv2.imshow('ridge_thresh', ridge_thresh)
        # cv2.waitKey(100)
        # cv2.imshow('ridge_thresh_R1', ridge_thresh_R1)
        # cv2.waitKey(100)
        cv2.imshow('ridge_thresh_R2', ridge_thresh_R2)
        cv2.waitKey(100)

        thinned_R2 = cv2.ximgproc.thinning(ridge_thresh_R2, thinningType= cv2.ximgproc.THINNING_ZHANGSUEN)
        cv2.imshow('thinned_R2', thinned_R2)
        cv2.waitKey(100)

        veins_only = cv2.bitwise_and(thinned_R2, thinned_R2, mask=erosion_resized)
        cv2.imshow('veins_only', veins_only)
        cv2.waitKey(100)

        cropped_veins = veins_only[((point1[1]//4)-50):((point2[1]//4)+50), ((point1[0]//4)-20):((point2[0]//4)+160)].copy()

        # cv2.rectangle(veins_only, ((point1[0]//4)-20,(point1[1]//4)-50), ((point2[0]//4)+150,(point2[1]//4)+50),(255),2)
        cv2.imshow('cropped_veins', cropped_veins)
        cv2.waitKey(100)

        cv2.imwrite('C:/Users/bryan/Desktop/College/FYP/Python Server/Hough/veins.jpg', cropped_veins)

        cropped_veins_BGR = cv2.merge([cropped_veins,cropped_veins,cropped_veins])

        lines = cv2.HoughLinesP(cropped_veins,rho=1,theta=numpy.pi/180,threshold=9,minLineLength=3, maxLineGap=6)
        # for line in lines:
        #     rho,theta = line[0]
        #     a = numpy.cos(theta)
        #     b = numpy.sin(theta)
        #     x0 = a*rho
        #     y0 = b*rho
        #     x1 = int(x0 + 1000*(-b))
        #     y1 = int(y0 + 1000*(a))
        #     x2 = int(x0 - 1000*(-b))
        #     y2 = int(y0 - 1000*(a))

        #     cv2.line(cropped_veins_BGR,(x1,y1),(x2,y2),(0,0,255),1)
        lines_polar = []
        for line in lines:
            x1,y1,x2,y2 = line[0]
            cv2.line(cropped_veins_BGR,(x1,y1),(x2,y2),(0,0,255),1)
            
            if (y2==y1):
                theta = 90
                rho = y2
            elif (x2==x1):
                theta = 0
                rho = x2
            else:
                m = (y2-y1)/(x2-x1)
                c = y2-(m*x2)

                theta = 90 + (numpy.arctan(m)*180/numpy.pi)
                rho = c*numpy.sin(theta)

            lines_polar.append([rho,theta])
        # print(lines)
        # print(lines_polar)
        # with open('C:/Users/bryan/Desktop/College/FYP/Python Server/Hough/veins.p','wb') as f:
        #     pickle.dump(lines_polar, f)
        
        cv2.imwrite('C:/Users/bryan/Desktop/College/FYP/Python Server/Hough/hough.jpg', cropped_veins_BGR)
        cv2.imshow('cropped_veins_BGR', cropped_veins_BGR)
        cv2.waitKey(0)
        
        
except KeyboardInterrupt:
    print('CTRL+C input detected. Closing Server Application...')
    My_Server.shutdown()
    sys.exit()
