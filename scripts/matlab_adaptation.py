#Matlab adaptation of code in python
#Blanchon Marc
# December 11, 2017
#
#Find correpondancies with the pattern to detect point of interest
#cluster points of interests

import numpy as np
from scipy import misc as MSC
import cv2
import matplotlib.pyplot as plt
from scipy.cluster import vq


#custom file
import cases as switch
#---------------------------------------------------------------------------------------------------------------------------
# FUNCTIONS
#---------------------------------------------------------------------------------------------------------------------------
def clustering(array):
    centroids, variance  = vq.kmeans(array, 3)
    return centroids

#---------------------------------------------------------------------------------------------------------------------------
# __MAIN__
#---------------------------------------------------------------------------------------------------------------------------

# def __main__:


exception = False
#safety check
try:
    imfile = "bwimage.png"
    im = cv2.imread(imfile, cv2.CV_LOAD_IMAGE_GRAYSCALE)
except:
    print("File is not an image")
    exception = True
#safety check
if not exception:
    if isinstance(im, np.ndarray):

        #set threshold and assure the black and white image
        thresh = 0
        im_bw = cv2.threshold(im, thresh, 1, cv2.THRESH_BINARY)[1]

        #Testing the values and display
        #cv2.namedWindow('win')
        #cv2.imshow('win', im_bw)
        #cv2.waitKey(0)

        b1, w1, b2, w2, b3 = 0 , 0 , 0 , 0 , 0

        #Testing the functions and output the result
        #checker = check_ratio(b1, w1, b2, w2, b3)
        #print checker

        startloc = np.array([0, 0])
        endloc = np.array([0, 0])
        detection = np.array([0,0])

        for nrow in range (0,im_bw.shape[0],2):
            #reset
            currState = 0
            b1, w1, b2, w2, b3 = 0 , 0 , 0 , 0 , 0
            pastFirstTest = False

            r0 = np.array([0, 0])
            r1 = np.array([0, 0])

            for ncol in range (0,im_bw.shape[1],2):

                    #ELEGANT SWITCH CASE
                    if currState == 0:
                        currState , b1 = switch.case_zero(im_bw , nrow , ncol , b1 , currState)
                    elif currState == 1:
                        currState , b1 , w1 = switch.case_one(im_bw , nrow , ncol , b1 , w1 , currState)
                    elif currState == 2:
                        currState , w1 , b2 , r0 = switch.case_two(im_bw , nrow , ncol , w1 , b2 , r0 , currState)
                    elif currState == 3:
                        currState, b2, w2 = switch.case_three(im_bw , nrow , ncol , b2 , w2 , currState)
                    elif currState == 4:
                        currState , pastFirstTest , w2 , b3 , r1 , r0 = switch.case_four(im_bw , nrow , ncol , w2 , b3 , r1 , r0 , currState , pastFirstTest)
                    elif currState == 5:
                        currState , b1 , w1 , b2 , w2 , b3 , r0 , pastFirstTest , detection , endloc , startloc = switch.case_five(im_bw , nrow , ncol , b1 , w1 , b2 , w2 , b3 , r0 , currState , pastFirstTest , detection , endloc , startloc)
                    else:
                        print("Current state index is out of bound")

#---------------------------------------------------------------------------------------------------------------------------
# DISPLAY
#---------------------------------------------------------------------------------------------------------------------------

        detection = np.delete(detection, (0), axis=0)

        implot = plt.imshow(im_bw)
        #detach x and y coordinated
        detect_x, detect_y = detection.T
        #display all the detection results
        plt.scatter(detect_y , detect_x)

        # put a red dot, size 40, at 2 locations:
        #plt.scatter(x=[30, 40], y=[50, 60], c='r', s=40)

        #clustering and extract the coordinates
        centroids = clustering(detection.astype(float))
        #detach x and y coordinated
        cengtroids_x, cengtroids_y = centroids.T
        #display centroid coordinates
        plt.scatter(cengtroids_y , cengtroids_x , c='r', s=20)


        plt.show()


    else:
        print("Image does not meet requirements")
